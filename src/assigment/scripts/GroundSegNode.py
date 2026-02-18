#!/usr/bin/python3
#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

class GroundSegNode:

    def __init__(self):
        rospy.init_node("planarity_ground_segmentation")

        self.voxel_size = 0.3
        self.planarity_threshold = 0.5

        self.grid_resolution = 0.3
        self.grid_size = 120

        rospy.Subscriber("/lidar",
                         PointCloud2,
                         self.cloud_callback,
                         queue_size=1)

        self.grid_pub = rospy.Publisher("/ground_grid",
                                        OccupancyGrid,
                                        queue_size=1)

    # --------------------------------------------------

    def cloud_callback(self, msg):
        points = self.pointcloud2_to_xyz(msg)

        if len(points) < 10:
            return

        voxel_dict = self.voxelize(points)
        grid = self.classify_voxels(voxel_dict)

        self.publish_grid(grid, msg.header)

    # --------------------------------------------------

    def pointcloud2_to_xyz(self, cloud_msg):
        points = []
        for p in pc2.read_points(cloud_msg,
                                 field_names=("x", "y", "z"),
                                 skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points)

    # --------------------------------------------------

    def voxelize(self, points):
        voxel_dict = {}

        indices = np.floor(points / self.voxel_size).astype(int)

        for idx, point in zip(indices, points):
            key = tuple(idx)
            voxel_dict.setdefault(key, []).append(point)

        return voxel_dict

    # --------------------------------------------------

    def compute_planarity(self, pts):
        pts = np.array(pts)

        if pts.shape[0] < 5:
            return 0

        cov = np.cov(pts.T)
        eigvals = np.linalg.eigvalsh(cov)

        eigvals = np.sort(eigvals)[::-1]  # λ1 ≥ λ2 ≥ λ3

        if eigvals[0] == 0:
            return 0

        planarity = (eigvals[1] - eigvals[2]) / eigvals[0]
        return planarity

    # --------------------------------------------------

    def classify_voxels(self, voxel_dict):

        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        origin = - (self.grid_size * self.grid_resolution) / 2.0

        for voxel_key, pts in voxel_dict.items():

            planarity = self.compute_planarity(pts)

            if planarity < self.planarity_threshold:
                continue

            # Use centroid for projection
            centroid = np.mean(pts, axis=0)

            x_idx = int((centroid[0] - origin) / self.grid_resolution)
            y_idx = int((centroid[1] - origin) / self.grid_resolution)

            if 0 <= x_idx < self.grid_size and 0 <= y_idx < self.grid_size:
                grid[x_idx, y_idx] = 0  # ground

        return grid

    # --------------------------------------------------

    def publish_grid(self, grid, header):

        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = - (self.grid_size *
                                        self.grid_resolution) / 2.0
        msg.info.origin.position.y = - (self.grid_size *
                                        self.grid_resolution) / 2.0

        msg.data = grid.flatten().tolist()

        self.grid_pub.publish(msg)


if __name__ == "__main__":
    GroundSegNode()
    rospy.spin()
