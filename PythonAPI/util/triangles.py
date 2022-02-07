import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from sklearn.cluster import DBSCAN
from collections import defaultdict
import argparse

def main(args):
    mesh = o3d.io.read_triangle_mesh(args.mesh)
    
    small_triangle = args.min_area



    #filter small triangles
    triangles = np.asarray(mesh.triangles).copy()
    vertices = np.asarray(mesh.vertices).copy()
    areas = np.ones_like(np.take(triangles, [0], axis=1)) 
    mask = np.zeros_like(np.take(triangles, [0], axis=1), dtype=bool)
    for index_triangle, t in enumerate(triangles):
        index_vertex = index_triangle * 3
        p1, p2, p3 = vertices[t[0]], vertices[t[1]], vertices[t[2]]

        # These two vectors are in the plane
        v1 = p3 - p1
        v2 = p2 - p1

        # the cross product is a vector normal to the plane
        cp = np.cross(v1, v2)
        if np.linalg.norm(cp) / 2 > small_triangle:
            areas[index_triangle] = np.linalg.norm(cp) / 2
            # This evaluates a * x3 + b * y3 + c * z3 which equals d
        else:
            mask[index_triangle] = True
    mesh.remove_triangles_by_mask(mask)
    # o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)


    

    #calculate planes
    mesh.compute_triangle_normals(normalized=True)
    normals = np.asarray(mesh.triangle_normals)
    
    triangles = np.asarray(mesh.triangles).copy()
    vertices = np.asarray(mesh.vertices).copy()

    planes = []
    for index_triangle, t in enumerate(triangles):
        d = np.dot(normals[index_triangle], vertices[t[2]])
        planes.append([*normals[index_triangle], d])
    planes = np.asarray(planes)



    clustering = DBSCAN(eps=1, min_samples=1, n_jobs=20).fit(planes)
    label_to_meshes = defaultdict(list)
    for label_id, label in enumerate(clustering.labels_):
        label_to_meshes[label].append(label_id)
    
    #construct meshes out of planes
    meshes = []
    triangles = np.asarray(mesh.triangles).copy()
    vertices = np.asarray(mesh.vertices).copy()
    for mesh_tr in label_to_meshes.values():
        print(len(mesh_tr))
        triangles_3 = []
        vertices_3 = []
        for i, triangle_id in enumerate(mesh_tr):
            t = triangles[triangle_id]
            vertices_3.append(vertices[t[0]])
            vertices_3.append(vertices[t[1]])
            vertices_3.append(vertices[t[2]])

            triangles_3.append(np.arange(i*3, i*3 + 3))
        new_mesh = o3d.geometry.TriangleMesh()
        new_mesh.triangles = o3d.utility.Vector3iVector(triangles_3)
        new_mesh.vertices = o3d.utility.Vector3dVector(vertices_3)
        if (new_mesh.get_surface_area() ==0 ):
            continue
        meshes.append(new_mesh)
    
    #sample and color mesh-planes
    pcds = []
    for mesh in meshes:
        pcd = mesh.sample_points_poisson_disk(int(np.sqrt(new_mesh.get_surface_area())/2))

        points = np.asarray(pcd.points)
        clustering = DBSCAN(eps=100, min_samples=200, n_jobs=20).fit(points)
        print(set(clustering.labels_))
        for label in set(clustering.labels_):
            new_pcd = o3d.geometry.PointCloud()
            new_points = []
            for i, lbl in enumerate(clustering.labels_):
                if lbl == label:
                    new_points.append(points[i])
            new_pcd.points = o3d.utility.Vector3dVector(np.array(new_points))
            pcds.append(new_pcd)
    colors = plt.cm.get_cmap('hsv', len(pcds))
    for color_id, pcd in enumerate(pcds):
        pcds[color_id].paint_uniform_color(colors(color_id)[:3]) 

    o3d.io.write_point_cloud(args.store, pcd)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(
        description=__doc__)

    argparser.add_argument(
        '--min_area',
        default=5000,
        help='the minimal area of triangles that will not be filtered')

    argparser.add_argument(
        '--mesh',
        help='path to mesh')

    argparser.add_argument(
        '--store',
        help='path to where to save pcd')

    args = argparser.parse_args()
    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')

