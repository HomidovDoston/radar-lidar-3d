import cv2
import torch
import numpy as np
import open3d as o3d
from PIL import Image
import matplotlib.pyplot as plt
from ultralytics import YOLO

# ======== Step 1: Fayl yo'lini belgilang ========
img_path = "your_image.png"  

# ======== Step 2: YOLOv8 bilan ob'ekt aniqlash ========
print("[INFO] Ob'ektlar aniqlanmoqda (YOLO)...")
model = YOLO('yolov8n.pt')
img_cv = cv2.imread(img_path)
results = model(img_cv)[0]
results.save(filename="yolo_result.jpg")
print("[INFO] YOLO natijasi 'yolo_result.jpg' sifatida saqlandi.")

# ======== Step 3: MiDaS orqali depth estimation ========
print("[INFO] MiDaS yuklanmoqda...")
midas = torch.hub.load("intel-isl/MiDaS", "DPT_Hybrid")
midas.eval()
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
midas.to(device)

# Load transforms
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = midas_transforms.dpt_transform

# Rasmni o'qish va RGB formatiga o'tkazish
img = cv2.imread(img_path)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_pil = Image.fromarray(img_rgb)

# Apply transform
input_tensor = transform(img_rgb).to(device)

# Ensure we have 4 dimensions (batch, channel, height, width)
if input_tensor.ndim == 3:
    input_tensor = input_tensor.unsqueeze(0)

print("[INFO] Chuqurlik xaritasi hisoblanmoqda...")
with torch.no_grad():
    prediction = midas(input_tensor)
    depth_map = prediction.squeeze().cpu().numpy()

# Depth xaritasini rasmning asl o'lchamlariga moslashtirish
original_h, original_w = img_rgb.shape[:2]
depth_map = cv2.resize(depth_map, (original_w, original_h))

plt.imsave("depth_map.png", depth_map, cmap='viridis')
print("[INFO] Depth xarita 'depth_map.png' sifatida saqlandi.")

# ======== Step 4: 3D nuqta buluti yaratish ========
def create_point_cloud(depth_map, image):
    h, w = depth_map.shape
    assert h == image.shape[0] and w == image.shape[1], "Depth map and image dimensions must match"
    
    fx = fy = 500  # Fokal uzunlik (oddiy qiymat)
    cx, cy = w // 2, h // 2

    points = []
    colors = []

    for y in range(h):
        for x in range(w):
            z = depth_map[y, x]
            if z <= 0:  # Yolg'on chuqurliklarni o'tkazib yuborish
                continue
            X = (x - cx) * z / fx
            Y = (y - cy) * z / fy
            points.append([X, Y, z])
            colors.append(image[y, x] / 255.0)

    return np.array(points), np.array(colors)

print("[INFO] Nuqta buluti yaratilyapti...")
points, colors = create_point_cloud(depth_map, img_rgb)

# ======== Step 5: 3D Vizualizatsiya ========
print("[INFO] 3D radar ko‘rinishida vizualizatsiya...")
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)


# if len(points) > 100000:
#     pcd = pcd.voxel_down_sample(voxel_size=0.01)

# Koordinata o'qlari yaratish
coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# Vizualizatsiya qilish uchun yangi kamera o'qlari va pozitsiyasini o'rnatish
vis = o3d.visualization.Visualizer()
vis.create_window(width=800, height=600)
vis.add_geometry(pcd)
vis.add_geometry(coord_axes)

# Kamera pozitsiyasini optimallashtirish
ctr = vis.get_view_control()
ctr.set_zoom(1.0)  # Zoom darajasini yaxshilash
ctr.set_front([0, 0, -1])  # To'g'ri ko'rish (kamera oldinga qarashi kerak)
ctr.set_up([0, -1, 0])  # Y o'qi yuqorida

# Vizualizatsiya
print("[INFO] Sichqoncha yordamida ko'rinishni burish, g'ildirak yordamida zoom qilish")
vis.run()
vis.destroy_window()
