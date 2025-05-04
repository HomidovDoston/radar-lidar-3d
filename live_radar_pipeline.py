import cv2
import torch
from PIL import Image
from ultralytics import YOLO
from midas.dpt_depth import DPTDepthModel
from midas.transforms import Resize, NormalizeImage, PrepareForNet
from torchvision import transforms
import numpy as np

# YOLO modelini yuklash
model = YOLO("yolov8n.pt")

# MiDaS modelini yuklash
midas_model = DPTDepthModel()
midas_model.load_state_dict(torch.load("midas_v21_small_256.pt"))
midas_model.eval()

# Transformlar
transform = transforms.Compose([
    Resize((384, 384)),
    NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    PrepareForNet()
])

# Kamera ochish
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera xato berdi.")
        break

    # YOLO modelini ishlatish
    results = model(frame)
    result_img = results[0].plot()

    # MiDaS uchun tayyorlash
    input_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_image = Image.fromarray(input_image)
    input_tensor = transform(input_image).unsqueeze(0).to(torch.float32)

    with torch.no_grad():
        prediction = midas_model(input_tensor)
        depth = prediction.squeeze().cpu().numpy()

    # Ko'rsatish
    cv2.imshow("YOLO", result_img)
    cv2.imshow("Depth", depth / np.max(depth))  # Depth ko'rinishi uchun normallashtirish

    # ESC bosilsa chiqish
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Tozalash
cap.release()
cv2.destroyAllWindows()


