import open3d as o3d
import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
import json
import time
from typing import List, Dict, Any
import cv2
import torch
from PIL import Image
from ultralytics import YOLO
import matplotlib.pyplot as plt

class SensorFusion3D:
    def __init__(self):
        # Dastur parametrlari
        self.voxel_size = 0.05
        self.dbscan_eps = 0.5
        self.dbscan_min_points = 10
        self.statistical_nb_neighbors = 20
        self.statistical_std_ratio = 2.0
        
        # Harakatni aniqlash uchun oldingi frame
        self.previous_point_cloud = None
        
        # 3D xarita
        self.global_map = o3d.geometry.PointCloud()
        
        # MiDaS modelini yuklash
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.midas = torch.hub.load("intel-isl/MiDaS", "DPT_Hybrid").to(self.device).eval()
        self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = self.midas_transforms.dpt_transform
        
        # YOLO modelini yuklash
        self.yolo_model = YOLO('yolov8n.pt')
    
    def load_lidar_data(self, file_path: str) -> o3d.geometry.PointCloud:
        """Lidar ma'lumotlarini yuklash"""
        if file_path.endswith('.pcd'):
            pcd = o3d.io.read_point_cloud(file_path)
        else:
            raise ValueError("Faqat .pcd formatidagi fayllar qo'llab-quvvatlanadi")
        return pcd
    
    def load_radar_data(self, file_path: str) -> pd.DataFrame:
        """Radar ma'lumotlarini yuklash"""
        return pd.read_csv(file_path)
    
    def preprocess_point_cloud(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Nuqtalar bulutini tozalash"""
        if not pcd.has_points():
            raise ValueError("Nuqtalar buluti bo'sh")
            
        # Statistik chiqindilarni olib tashlash
        cl, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.statistical_nb_neighbors,
            std_ratio=self.statistical_std_ratio
        )
        
        # Voxel orqali sizlarni kamaytirish
        downpcd = cl.voxel_down_sample(voxel_size=self.voxel_size)
        
        return downpcd
    
    def cluster_point_cloud(self, pcd: o3d.geometry.PointCloud) -> List[o3d.geometry.PointCloud]:
        """Nuqtalar bulutini klasterlarga ajratish"""
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return []
        
        db = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_points).fit(points)
        labels = db.labels_
        
        clusters = []
        for label in np.unique(labels):
            if label == -1:  # Noise
                continue
            cluster_points = points[labels == label]
            cluster_pcd = o3d.geometry.PointCloud()
            cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
            clusters.append(cluster_pcd)
                
        return clusters
    
    def create_bounding_boxes(self, clusters: List[o3d.geometry.PointCloud]) -> List[o3d.geometry.OrientedBoundingBox]:
        """Har bir klaster uchun bounding box yaratish"""
        boxes = []
        for cluster in clusters:
            if len(cluster.points) < 3:  # Kamida 3 nuqta bo'lishi kerak
                continue
            box = cluster.get_oriented_bounding_box()
            box.color = [1, 0, 0]  # Qizil rang
            boxes.append(box)
        return boxes
    
    def calculate_distances(self, clusters: List[o3d.geometry.PointCloud], 
                          sensor_position: np.ndarray = np.zeros(3)) -> List[float]:
        """Har bir obyekt markazidan sensorgacha bo'lgan masofani hisoblash"""
        distances = []
        for cluster in clusters:
            points = np.asarray(cluster.points)
            center = np.mean(points, axis=0)
            distance = np.linalg.norm(center - sensor_position)
            distances.append(distance)
        return distances
    
    def detect_moving_objects(self, current_pcd: o3d.geometry.PointCloud, 
                            threshold: float = 0.1) -> o3d.geometry.PointCloud:
        """Harakatlanuvchi obyektlarni aniqlash"""
        if self.previous_point_cloud is None:
            self.previous_point_cloud = current_pcd
            return None
        
        # ICP registratsiyasi
        reg_result = o3d.pipelines.registration.registration_icp(
            current_pcd, self.previous_point_cloud, threshold,
            np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        
        # Harakatdagi nuqtalarni aniqlash
        current_points = np.asarray(current_pcd.points)
        prev_points = np.asarray(self.previous_point_cloud.points)
        
        if len(current_points) == 0 or len(prev_points) == 0:
            return None
        
        # KNN orqali eng yaqin nuqtalarni topish
        from sklearn.neighbors import NearestNeighbors
        nbrs = NearestNeighbors(n_neighbors=1).fit(prev_points)
        distances, _ = nbrs.kneighbors(current_points)
        
        moving_indices = np.where(distances > threshold)[0]
        
        moving_pcd = o3d.geometry.PointCloud()
        moving_pcd.points = o3d.utility.Vector3dVector(current_points[moving_indices])
        
        self.previous_point_cloud = current_pcd
        return moving_pcd
    
    def update_global_map(self, pcd: o3d.geometry.PointCloud):
        """Global xaritani yangilash"""
        if not pcd.has_points():
            return
            
        combined_points = np.vstack([np.asarray(self.global_map.points), 
                                   np.asarray(pcd.points)])
        self.global_map.points = o3d.utility.Vector3dVector(combined_points)
        
        if len(self.global_map.points) > 10000:
            self.global_map = self.global_map.voxel_down_sample(voxel_size=self.voxel_size)
    
    def visualize_results(self, pcd: o3d.geometry.PointCloud, 
                        clusters: List[o3d.geometry.PointCloud], 
                        boxes: List[o3d.geometry.OrientedBoundingBox], 
                        moving_objects: o3d.geometry.PointCloud = None):
        """Natijalarni vizualizatsiya qilish"""
        vis_elements = [pcd]
        
        # Klasterlarni ranglash
        colors = plt.get_cmap("tab20")(np.linspace(0, 1, len(clusters)))[:, :3]
        for i, cluster in enumerate(clusters):
            cluster.paint_uniform_color(colors[i])
            vis_elements.append(cluster)
        
        # Bounding boxlarni qo'shish
        vis_elements.extend(boxes)
        
        # Harakatdagi obyektlarni ko'rsatish
        if moving_objects and moving_objects.has_points():
            moving_objects.paint_uniform_color([0, 1, 0])  # Yashil rang
            vis_elements.append(moving_objects)
        
        # Global xaritani ko'rsatish
        if self.global_map.has_points():
            self.global_map.paint_uniform_color([0.5, 0.5, 0.5])  # Kulrang rang
            vis_elements.append(self.global_map)
        
        # Vizualizatsiya oynasini sozlash
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1280, height=720)
        
        for geom in vis_elements:
            vis.add_geometry(geom)
        
        # Kamera parametrlari
        view_control = vis.get_view_control()
        view_control.set_front([0, -1, -0.5])  # Yukaridan ko'rinish
        view_control.set_up([0, -1, 0])       # Y o'qi tepada
        view_control.set_zoom(0.8)
        
        # Render qilish
        vis.run()
        vis.destroy_window()
    
    def generate_json_output(self, clusters: List[o3d.geometry.PointCloud], 
                           distances: List[float]) -> Dict[str, Any]:
        """Natijalarni JSON formatida chiqarish"""
        output = {
            "timestamp": time.time(),
            "objects": []
        }
        
        for i, (cluster, distance) in enumerate(zip(clusters, distances)):
            points = np.asarray(cluster.points)
            if len(points) == 0:
                continue
                
            center = np.mean(points, axis=0).tolist()
            size = (np.max(points, axis=0) - np.min(points, axis=0)).tolist()
            
            output["objects"].append({
                "id": i,
                "center": center,
                "size": size,
                "distance": float(distance),
                "point_count": len(points)
            })
        
        return output
    
    def process_frame(self, img_path: str = None, lidar_path: str = None, radar_path: str = None):
        """Asosiy ishlov berish metod"""
        processed_pcd = None
        
        if img_path:
            img = cv2.imread(img_path)
            if img is None:
                raise ValueError(f"Rasmni yuklab bo'lmadi: {img_path}")
                
            # Chuqurlik xaritasini yaratish
            depth_map = self.estimate_depth(img)
            pcd_from_img = self.convert_depth_to_3d(depth_map)
            
            if processed_pcd is None:
                processed_pcd = pcd_from_img
            else:
                # Nuqtalarni birlashtirish
                combined_points = np.vstack([
                    np.asarray(processed_pcd.points),
                    np.asarray(pcd_from_img.points)
                ])
                processed_pcd.points = o3d.utility.Vector3dVector(combined_points)
        
        if lidar_path:
            pcd = self.load_lidar_data(lidar_path)
            if processed_pcd is None:
                processed_pcd = self.preprocess_point_cloud(pcd)
            else:
                # Lidar nuqtalarini qo'shish
                lidar_pcd = self.preprocess_point_cloud(pcd)
                combined_points = np.vstack([
                    np.asarray(processed_pcd.points),
                    np.asarray(lidar_pcd.points)
                ])
                processed_pcd.points = o3d.utility.Vector3dVector(combined_points)
        
        if radar_path:
            radar_data = self.load_radar_data(radar_path)
            # Radar ma'lumotlarini qo'shish (soddalashtirilgan)
            if not radar_data.empty:
                radar_points = radar_data[['x', 'y', 'z']].values
                if len(radar_points) > 0:
                    radar_pcd = o3d.geometry.PointCloud()
                    radar_pcd.points = o3d.utility.Vector3dVector(radar_points)
                    
                    if processed_pcd is None:
                        processed_pcd = radar_pcd
                    else:
                        combined_points = np.vstack([
                            np.asarray(processed_pcd.points),
                            radar_points
                        ])
                        processed_pcd.points = o3d.utility.Vector3dVector(combined_points)
        
        if processed_pcd is None or not processed_pcd.has_points():
            raise ValueError("Hech qanday ma'lumot yuklanmadi")
        
        # Klasterlash va vizualizatsiya
        clusters = self.cluster_point_cloud(processed_pcd)
        boxes = self.create_bounding_boxes(clusters)
        distances = self.calculate_distances(clusters)
        moving_objects = self.detect_moving_objects(processed_pcd)
        self.update_global_map(processed_pcd)
        
        # Natijalarni ko'rsatish
        self.visualize_results(processed_pcd, clusters, boxes, moving_objects)
        
        # JSON chiqish
        json_output = self.generate_json_output(clusters, distances)
        print(json.dumps(json_output, indent=4))
    
    # def estimate_depth(self, img: np.ndarray) -> np.ndarray:
    #     """Rasmni chuqurlik xaritasiga aylantirish"""
    #     img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #     input_tensor = self.transform(img_rgb).to(self.device)
        
    #     with torch.no_grad():
    #         prediction = self.midas(input_tensor.unsqueeze(0))
    #         depth_map = prediction.squeeze().cpu().numpy()
        
    #     return depth_map
    
    
    def convert_depth_to_3d(self, depth_map: np.ndarray) -> o3d.geometry.PointCloud:
        """Chuqurlik xaritasidan 3D nuqtalar bulutini yaratish"""
        height, width = depth_map.shape
        fx = fy = 0.5 * width  # Fokal uzunlik
        
        # Normalizatsiya va masshtablash
        depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
        depth_map = depth_map * 10  # Masshtablash
        
        points = []
        for y in range(0, height, 2):  # Har 2-pikseldan olish
            for x in range(0, width, 2):
                z = depth_map[y, x]
                if z <= 0.1:  # Juda yaqin nuqtalarni o'tkazib yuborish
                    continue
                
                # Piksel koordinatalarini 3D ga o'tkazish
                X = (x - width/2) * z / fx
                Y = (y - height/2) * z / fy
                points.append([X, Y, z])
        
        point_cloud = o3d.geometry.PointCloud()
        if len(points) > 0:
            point_cloud.points = o3d.utility.Vector3dVector(np.array(points))
        
        return point_cloud

# Foydalanish misoli
if __name__ == "__main__":
    fusion = SensorFusion3D()
    
    # Faqat rasm orqali ishlash
    fusion.process_frame(img_path="your_image.png")
    
    # Yoki barcha sensorlardan ma'lumotlar bilan ishlash
    # fusion.process_frame(
    #     img_path="test_image.jpg",
    #     lidar_path="data.pcd",
    #     radar_path="radar.csv"
    # )