import glob
import os
import readline
import sys
import argparse
import time

import argparse
from datetime import datetime
import random
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import ast
# Check https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera for label explanation
good_labels = set([0, 1]) 

def main(args):
    with open(args.path) as f:
        np_pts = np.empty((0,3))
        filtered_ids = []
        filtered = []
        for i in range(args.lines):
            line = f.readline()
            if not line:
                break
            sep = line.split(",|,")
            tr = sep[0] #transformation to world coordinates
            points_packed = ast.literal_eval(sep[1])
            
            print(i)
            points = [point_label_id[0] for point_label_id in points_packed]
            ids = [point_label_id[2] for point_label_id in points_packed]
            labels = [point_label_id[1] for point_label_id in points_packed]
            # TODO USE NP FUNCTIONS!
            for i, label in enumerate(labels):
                if label in good_labels:
                    filtered.append(points[i])
                    filtered_ids.append(ids[i])
        id_types = set(filtered_ids)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(filtered))
        o3d.io.write_point_cloud(args.store, pcd)

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)

    argparser.add_argument(
        '--lines',
        type=int,
        default=10000000,
        help='the number of pointclouds to be united')

    argparser.add_argument(
        '--path',
        help='path to measurements file')

    argparser.add_argument(
        '--store',
        help='path to where to save new pcd')

    args = argparser.parse_args()
    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
