import cv2

camera_url = "http://192.168.31.218:8080/video"  # Telefon IP-camera URL

cap = cv2.VideoCapture(camera_url)

print("OpenCV versiyasi:", cv2.__version__)

if not cap.isOpened():
    print("Kamera ochilmadi.")
    exit()

print("Kamera muvaffaqiyatli ochildi.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kadr olinmadi.")
        break

    cv2.imshow("Phone Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

