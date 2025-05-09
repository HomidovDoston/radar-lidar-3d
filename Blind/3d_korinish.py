import open3d as o3d
import numpy as np

# Simulyatsiya uchun 3D nuqtalar (x, y, z)
points = np.random.rand(1000, 3) * 2  # 2m doirada

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

o3d.visualization.draw_geometries([pcd])
