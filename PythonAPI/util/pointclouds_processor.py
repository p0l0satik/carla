import glob
import os
import readline
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import ast
# Check https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera for label explanation
# good_labels = set([1, 2, 7, 8, 11, 17, ])
good_labels = set([1])


if __name__ == "__main__":
    ##alligns a argv[2] number of pointcloud recordings of lidar taken from file argv[2]
    with open(sys.argv[1]) as f:
        np_pts = np.empty((0,3))
        filtered_ids = []
        filtered = []
        for i in range(int(sys.argv[2])):
            line = f.readline()
            sep = line.split(")),")
            tr = sep[0]
            points_packed = ast.literal_eval(sep[1])
            
            print(ast.literal_eval(sep[1])[0])
            points = [point_label_id[0] for point_label_id in points_packed]
            ids = [point_label_id[2] for point_label_id in points_packed]
            labels = [point_label_id[1] for point_label_id in points_packed]
            # filtered = []
            # TODO USE NP FUNCTIONS!
            for i, label in enumerate(labels):
                if label in good_labels:
                    filtered.append(points[i])
                    filtered_ids.append(ids[i])
        id_types = set(filtered_ids)
        print(np_pts.shape, len(filtered_ids))

        objects_pcds = []

        for obj in id_types:
            obj_points = []
            for i, p in enumerate(filtered):
                if (filtered_ids[i] == obj):
                    obj_points.append(p)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(obj_points))
            objects_pcds.append(pcd)
    ###################
    #Uses ransac to segment pointcloud
        # min_pcd, min_plane = 1000, 100
        # for pcd in objects_pcds:
        #     planes = []
        #     ids  = []
        #     # pcd = pcd.voxel_down_sample(0.2)
        #     pcd.paint_uniform_color([0.3, 0.3, 0.3])

        #     tiny_plane = False
        #     print(np.asarray(pcd.points).shape[0])
        #     while len(planes) < 10 and not tiny_plane and (np.asarray(pcd.points).shape[0] > min_pcd) :
        #         plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
        #                                     ransac_n=3,
        #                                     num_iterations=1000)
        #         if (len(inliers) > min_plane):
        #             planes.append(pcd.select_by_index(inliers))
        #             ids.append(len(planes))
        #         else:
        #             tiny_plane = True
                
        #         pcd = pcd.select_by_index(inliers, invert=True)
                
                
        #     if len(ids) == 0:
        #         continue
        #     unq_ids, cnt = np.unique(ids, return_counts=True)
        #     colors = plt.cm.get_cmap('jet', len(unq_ids))
        #     print(colors(0))
        #     for color_id, unq_id in enumerate(unq_ids):
        #         print(colors(color_id))
        #         planes[color_id].paint_uniform_color(colors(color_id)[:3]) 
        #     planes.append(pcd)
        # o3d.visualization.draw_geometries(objects_pcds)
    #####################
    # an attempt to use coordinates to segment pointcloud
        mesh = o3d.io.read_triangle_mesh("../examples/SM_MERGED_SM_Museum_v3_2505.obj")
        mesh2 = o3d.io.read_triangle_mesh("../examples/SM_MERGED_ProceduralBuilding_26.obj")

        
        pcd = mesh.sample_points_poisson_disk(200)
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) / 100)
        transform_arr = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [137.522, 22.8169, 1.9611, 1]]
        pcd = pcd.transform(np.array(transform_arr).T)
        R = np.array([
            1, 0, 0,
            0, 0, 1,
            0, 1, 0
        ]).reshape((3, 3))
        pcd.rotate(R)
        # pcd = pcd.translate(np.array([13752.193, 2281.695, 196.110]))
        # o3d.visualization.draw_geometries([pcd])

        objects_pcds.append(pcd)

        pcd2 = mesh2.sample_points_poisson_disk(200)
        pcd2.points = o3d.utility.Vector3dVector(np.asarray(pcd2.points) / 100)

        transform_arr2 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [13.75, 0.15, -46.95, 1]]
        pcd2 = pcd2.transform(np.array(transform_arr2).T)
        R = np.array([
            1, 0, 0,
            0, 0, 1,
            0, 1, 0
        ]).reshape((3, 3))
        pcd2.rotate(R)

        # pcd = pcd.translate(np.array([13752.193, 2281.695, 196.110]))
        # o3d.visualization.draw_geometries([pcd])

        objects_pcds.append(pcd2)
        o3d.visualization.draw_geometries(objects_pcds)



