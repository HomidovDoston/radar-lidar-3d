import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

SERIAL_PORT = '/dev/ttyUSB0'  # Yoki 'COM3' (Windows)
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
print("Serial port ochildi:", SERIAL_PORT)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_vals, y_vals, z_vals = [], [], []

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        try:
            x, y, z = map(float, line.split(','))
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)

            if len(x_vals) > 1000:
                x_vals = x_vals[-1000:]
                y_vals = y_vals[-1000:]
                z_vals = z_vals[-1000:]

            ax.clear()
            ax.scatter(x_vals, y_vals, z_vals, c='b', s=1)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.pause(0.001)
        except ValueError:
            continue
except KeyboardInterrupt:
    print("To‘xtatildi.")
    ser.close()
