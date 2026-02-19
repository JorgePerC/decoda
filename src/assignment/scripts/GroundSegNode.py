#!/usr/bin/python3

import rospy 

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from message_filters import TimeSynchronizer, Subscriber

import math
import numpy as np
   
def voxelAndNormalCallback(msgNormal, msgVoxel):

    # Transform the point cloud into an xyz space
    pclNormals = list(pc2.read_points(msgNormal, field_names=("normal_x","normal_y","normal_z", "curvature"), skip_nans=True))
    pclVoxels = list(pc2.read_points(msgVoxel, field_names=("x","y","z", "intensity", "tag", "line", "timestamp"), skip_nans=True))


    rospy.loginfo("Sizes: Normal {}, Voxel {}".format(len(pclNormals), len(pclVoxels)))

    sizePublisher.publish( "Normal size {}. Voxel size {}".format(len(pclNormals), len(pclVoxels)))

    # Empty new PC2
    ground_points = []
    obstacle_points = []

    for point in pclNormals:
        nx, ny, nz, curvature = point
        # Suggested by LLM, to avoid numerical issues with arctan2 when normalVec[2] is close to zero
        nz_clamped = max(-1.0, min(1.0, nz))
        # Absolute tilt ignoring direction
        horizontal = math.sqrt(nx**2 + ny**2)
        tilt = math.atan2(horizontal, abs(nz_clamped))
        tilt_deg = math.degrees(tilt)

        # Segment ground and obstacles based on normal angles
        if tilt_deg < 10:  # Threshold for ground (adjust as needed)
            ground_points.append(point)
        else:
            obstacle_points.append(point)

    # Publish ground points

    # #header
    # newHeader = Header()
    # newHeader.stamp = rospy.Time.now()
    # newHeader.frame_id = 'your_frame'

    # groundCloud = pc2.create_cloud_xyz32(newHeader, ground_points)
    # ground_pub.publish(groundCloud)

    # # Publish obstacle points
    # obstacleCloud = pc2.create_cloud_xyz32(newHeader, obstacle_points)
    # obstacle_pub.publish(obstacleCloud)


def main():
    rospy.init_node('ObstacleSegmentator')
    
    # Subscribers and publishers
    normals_sub = Subscriber('/normal_estimation/output', PointCloud2)
    voxel_sub = Subscriber('/voxel_grid/output', PointCloud2)

    # Since we want to publish the segmentd point clouds, the input of
    # boxel and normal topics are required. Not only that, but also in sync, 
    # so we have the same dimention of their respective point clouds.
    # And can later match their xyz position to the slope of the normals.
    tss = TimeSynchronizer([normals_sub, voxel_sub], 10)
    tss.registerCallback(voxelAndNormalCallback)

    sizePublisher = rospy.Publisher('gndSize', String, queue_size=10)

#   ground_pub = rospy.Publisher("/ground_grid", PointCloud2, queue_size=1)
#   obstacle_pub = rospy.Publisher("/obstacle_grid", PointCloud2, queue_size=1)

    # This would basically keet the node alive, as long as there are messages in the topic
    rospy.spin()

if __name__ == '__main__':
  main()