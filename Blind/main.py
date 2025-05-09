import serial
import pyttsx3
import open3d as o3d
import numpy as np

engine = pyttsx3.init()
ser = serial.Serial('COM3', 9600)

points = []

def speak(text):
    engine.say(text)
    engine.runAndWait()

while True:
    line = ser.readline().decode().strip()
    if "DIST" in line:
        try:
            parts = line.split(',')
            dist = int(parts[0].split(':')[1])
            temp = float(parts[1].split(':')[1])
            shape = parts[3].split(':')[1]

            print(f"Distance: {dist} cm, Temp: {temp}, Shape: {shape}")

            # TTS
            if dist < 150 and temp > 30:
                speak(f"{shape} jism oldinda. Masofa {dist / 100:.1f} metr.")

            # 3D nuqta (mock)
            x, y, z = np.random.rand(3) * 2  # Simulyatsiya uchun
            points.append([x, y, z])

            if len(points) > 500:
                points = points[-500:]  # So‘ngi 500 nuqtani saqlash

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(points))
                o3d.visualization.draw_geometries([pcd])

        except Exception as e:
            print("Xato:", e)
