# import open3d as o3d
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# def generate_synthetic_point_cloud():
#     print("[INFO] Sun’iy nuqta buluti yaratilmoqda...")
#     points = np.random.uniform(-5, 5, size=(1000, 3))
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     o3d.io.write_point_cloud("lidar_data.pcd", pcd)
#     print("[INFO] Fayl 'lidar_data.pcd' nomi bilan saqlandi.")
#     return pcd

# def load_point_cloud(filename):
#     print(f"[INFO] '{filename}' fayli o‘qilmoqda...")
#     pcd = o3d.io.read_point_cloud(filename)
#     return pcd

# def filter_point_cloud(pcd, voxel_size=0.1):
#     print("[INFO] Nuqta buluti filtrlanyapti...")
#     return pcd.voxel_down_sample(voxel_size=voxel_size)

# def segment_ground(pcd):
#     print("[INFO] Tekislik aniqlanmoqda (RANSAC)...")
#     plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
#                                              ransac_n=3,
#                                              num_iterations=1000)
#     ground = pcd.select_by_index(inliers)
#     objects = pcd.select_by_index(inliers, invert=True)
#     return ground, objects

# def cluster_objects(pcd):
#     print("[INFO] Ob’ektlar klassterlanmoqda...")
#     labels = np.array(pcd.cluster_dbscan(eps=0.3, min_points=10, print_progress=True))
#     max_label = labels.max()
#     print(f"[INFO] Topilgan klassterlar soni: {max_label + 1}")
    
#     colors = plt.get_cmap("tab20")(labels / (max_label + 1 if max_label >= 0 else 1))
#     colors[labels < 0] = [0, 0, 0, 1]
#     return np.asarray(pcd.points), labels, colors

# def visualize_with_matplotlib(points, labels, colors):
#     print("[INFO] Matplotlib orqali 3D chizma yaratilmoqda...")
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=5)
#     ax.set_title("3D Lidar Point Cloud - Clustered")
#     plt.show()

# if __name__ == "__main__":
#     generate_synthetic_point_cloud()
#     pcd = load_point_cloud("lidar_data.pcd")
#     pcd_filtered = filter_point_cloud(pcd)
#     ground, objects = segment_ground(pcd_filtered)
#     points, labels, colors = cluster_objects(objects)
#     visualize_with_matplotlib(points, labels, colors)

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_radar_point_cloud(num_points=1000, max_radius=10):
    print("[INFO] Radar uslubida sun’iy nuqta buluti yaratilmoqda...")
    azimuth = np.random.uniform(0, 2 * np.pi, num_points)
    elevation = np.random.uniform(-np.pi/8, np.pi/8, num_points)  # 45° sektor
    radius = np.random.uniform(1, max_radius, num_points)

    # Sferik koordinatalarni (r, θ, φ) → (x, y, z)
    x = radius * np.cos(elevation) * np.cos(azimuth)
    y = radius * np.cos(elevation) * np.sin(azimuth)
    z = radius * np.sin(elevation)

    points = np.vstack((x, y, z)).T
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud("radar_style_data.pcd", pcd)
    print("[INFO] 'radar_style_data.pcd' nomi bilan saqlandi.")
    return pcd

def load_point_cloud(filename):
    print(f"[INFO] '{filename}' fayli o‘qilmoqda...")
    return o3d.io.read_point_cloud(filename)

def cluster_objects(pcd):
    print("[INFO] Ob’ektlar klassterlanmoqda...")
    labels = np.array(pcd.cluster_dbscan(eps=0.5, min_points=10, print_progress=True))
    max_label = labels.max()
    print(f"[INFO] Topilgan klassterlar soni: {max_label + 1}")
    
    colors = plt.get_cmap("tab20")(labels / (max_label + 1 if max_label >= 0 else 1))
    colors[labels < 0] = [0, 0, 0, 1]
    return np.asarray(pcd.points), labels, colors

def visualize_with_matplotlib(points, labels, colors):
    print("[INFO] Matplotlib orqali radar ko‘rinishida 3D chizma yaratilmoqda...")
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=5)
    ax.set_title("Radar Ko‘rinishida 3D Nuqta Buluti")
    ax.set_xlabel('X o‘qi')
    ax.set_ylabel('Y o‘qi')
    ax.set_zlabel('Z o‘qi')
    plt.show()

if __name__ == "__main__":
    pcd = generate_radar_point_cloud()
    pcd_loaded = load_point_cloud("radar_style_data.pcd")
    points, labels, colors = cluster_objects(pcd_loaded)
    visualize_with_matplotlib(points, labels, colors)
