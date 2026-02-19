#!/usr/bin/python3

import rospy

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from message_filters import ApproximateTimeSynchronizer, Subscriber

import math

class GroundSegNode:

    def __init__(self):
        rospy.init_node('ObstacleSegmentator')
        
        # Subscribers and publishers
        normals_sub = Subscriber('/normal_estimation/output', PointCloud2)
        voxel_sub = Subscriber('/voxel_grid/output', PointCloud2)

        # Since we want to publish the segmentd point clouds, the input of
        # boxel and normal topics are required. Not only that, but also in sync, 
        # so we have the same dimention of their respective point clouds.
        # And can later match their xyz position to the slope of the normals.
        tss = ApproximateTimeSynchronizer([normals_sub, voxel_sub], 10, slop=0.05)
        tss.registerCallback(self.voxelAndNormalCallback)

        self.ground_pub = rospy.Publisher("/ground_grid", PointCloud2, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacle_grid", PointCloud2, queue_size=1)

    def voxelAndNormalCallback(self, msgNormal, msgVoxel):
        rospy.loginfo("voxelAndNormalCallback called")

        # Transform the point cloud into an xyz space
        # It's really import to not skip nans, as they are used to fill the empty spaces in the voxel grid, and we need them to keep the same dimension
        pclNormals = list(pc2.read_points(
            msgNormal,
            field_names=("normal_x","normal_y","normal_z"),
            skip_nans=False
        ))

        pclVoxels = list(pc2.read_points(
            msgVoxel,
            field_names=("x","y","z"),
            skip_nans=False
        ))

        #Check for same dimension
        rospy.loginfo("Sizes: Normal {}, Voxel {}".format(len(pclNormals), len(pclVoxels)))
        if len(pclNormals) != len(pclVoxels):
            rospy.logerr("Cloud size mismatch — aborting frame")
            return

        
        # Empty new PC2
        ground_points = []
        obstacle_points = []

        # Iterate through the normals and voxels
        for i in range(len(pclNormals)):
            nx,ny,nz = pclNormals[i]
            x,y,z    = pclVoxels[i]

            if any(math.isnan(v) for v in (nx,ny,nz,x,y,z)):
                continue
        
            # Absolute tilt ignoring direction
            horizontal = math.sqrt(nx**2 + ny**2)
            tilt = math.atan2(horizontal, abs(nz))
            tilt_deg = math.degrees(tilt)

            # Segment ground and obstacles based on normal angles
            if tilt_deg < 20:  # Threshold for ground (adjust as needed)
                ground_points.append((x, y, z))
            else:
                obstacle_points.append((x, y, z))

        if not ground_points:
            rospy.logwarn("No ground detected in this frame")
        else:
            # Publish ground points
            groundCloud = pc2.create_cloud_xyz32(msgVoxel.header, ground_points)
            self.ground_pub.publish(groundCloud)
        if not obstacle_points:
            rospy.logwarn("No obstacles detected in this frame")
        else:
            # Publish obstacle points
            obstacleCloud = pc2.create_cloud_xyz32(msgVoxel.header, obstacle_points)
            self.obstacle_pub.publish(obstacleCloud)


    def run(self):
        # This would basically keet the node alive, as long as there are messages in the topic
        rospy.spin()
if __name__ == '__main__':

    try:
        node = GroundSegNode()
        node.run()
    except rospy.ROSInterruptException:
        pass