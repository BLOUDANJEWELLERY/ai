import open3d as o3d
import numpy as np

def scale_point_cloud(pc, px_per_mm):
    return pc / px_per_mm

def reconstruct_mesh(point_cloud):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd = pcd.voxel_down_sample(voxel_size=1.0)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh = mesh.crop(bbox)

    return mesh

def calculate_volume(mesh):
    mesh.compute_vertex_normals()
    try:
        volume = mesh.get_volume()
    except:
        volume = mesh.get_oriented_bounding_box().volume()
    return volume
