#!/usr/bin/python3

import rospy 

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

import numpy as np
   
def pointCldCallback( msg):

    # Transform the point cloud into an xyz space

    pclNormals = list(pc2.read_points(msg, field_names=("x", "y", "z","normal_x","normal_y","normal_z", "curvature"), skip_nans=True))
    xyzSpace = np.array(pclNormals)

    for point in xyzSpace:


def main():
    rospy.init_node('ObstacleSegmentator')
    
    # Subscribers and publishers
    rospy.Subscriber('/normal_estimation/output', PointCloud2, pointCldCallback, queue_size=1)

    ground_pub = rospy.Publisher("/ground_grid", PointCloud2, queue_size=1)
    obstacle_pub = rospy.Publisher("/obstacle_grid", PointCloud2, queue_size=1)

    # This would basically keet the node alive, as long as there are messages in the topic
    rospy.spin()

if __name__ == '__main__':
  main()