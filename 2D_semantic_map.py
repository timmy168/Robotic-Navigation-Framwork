import numpy as np
import open3d as o3d
import argparse
import os
import copy
import time
import math
from sklearn.neighbors import NearestNeighbors
import cv2
import matplotlib.pyplot as plt

class Map:
    def __init__(self, point_path, color_path):
        self.point_path = point_path
        self.color_path = color_path
        self.points = np.load(self.point_path)
        self.colors = np.load(self.color_path)

    def get_pcd(self, pcd_path):
        pcd = o3d.io.read_point_cloud(pcd_path)
        o3d.visualization.draw_geometries([pcd])
        return pcd

    def construct_pcd(self):
        pcd = o3d.geometry.PointCloud()
        self.points = self.points * 10000 / 255
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.colors)
        # o3d.visualization.draw_geometries([pcd])

        # filter the ceiling and the floor
        xyz_points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        ceiling_y = 0.0135
        floor_y = -1.35
        filtered_xyz_points = xyz_points[(xyz_points[:, 1] <= ceiling_y) 
                                        &(  floor_y <= xyz_points[:, 1])]
        filtered_colors = colors[(xyz_points[:, 1] <= ceiling_y) 
                                &(  floor_y <= xyz_points[:, 1])]
        pcd.points = o3d.utility.Vector3dVector(filtered_xyz_points)
        pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
        o3d.visualization.draw_geometries([pcd])
        return pcd
    
    def construct_image(self, pcd):
        # create scatter plot
        px = 1/plt.rcParams['figure.dpi']
        plt.figure(figsize=(1700 * px, 1100 * px))
        points = np.asarray(pcd.points)
        plt.scatter(points[:, 2], points[:, 0],s=5, c = np.asarray(pcd.colors), marker='o')

        # Set the scale
        plt.xlim(points[:, 2].min(), points[:, 2].max())
        plt.ylim(points[:, 0].min(), points[:, 0].max())
        plt.axis('off')
        plt.savefig('map.png', bbox_inches = 'tight', pad_inches = 0)

        plt.show()

if __name__ == '__main__':
    point_path = "semantic_3d_pointcloud/point.npy"
    color_path = "semantic_3d_pointcloud/color01.npy"
    semantic_map = Map(point_path,color_path)
    pcd = semantic_map.construct_pcd()
    map_points = np.asarray(pcd.points)
    map_colors = np.asarray(pcd.colors)
    np.save("2D_semantic_map_points.npy",map_points)
    np.save("2D_semantic_map_colors.npy",map_colors)
    semantic_map.construct_image(pcd)