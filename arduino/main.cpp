#include <Wire.h>
#include <Servo.h>

#define LIDAR_ADDR 0x62
#define REGISTER_ACQ_COMMAND 0x00
#define REGISTER_DISTANCE 0x8f
#define ACQ_COMMAND 0x04
#define DEG_TO_RAD 0.0174533

Servo panServo;
Servo tiltServo;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  panServo.attach(9);   // Pan servo
  tiltServo.attach(10); // Tilt servo
  delay(1000);
}

void loop() {
  for (int pan = 0; pan <= 180; pan += 10) {
    panServo.write(pan);
    delay(300);

    for (int tilt = 60; tilt <= 120; tilt += 10) {
      tiltServo.write(tilt);
      delay(300);

      int distance = readLidar();
      if (distance > 0 && distance < 4000) {
        float panRad = pan * DEG_TO_RAD;
        float tiltRad = tilt * DEG_TO_RAD;

        float x = distance * sin(tiltRad) * cos(panRad);
        float y = distance * sin(tiltRad) * sin(panRad);
        float z = distance * cos(tiltRad);

        Serial.print(x, 2); Serial.print(",");
        Serial.print(y, 2); Serial.print(",");
        Serial.println(z, 2);
      }
    }
  }
  delay(2000);
}

int readLidar() {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(REGISTER_ACQ_COMMAND);
  Wire.write(ACQ_COMMAND);
  Wire.endTransmission();
  delay(20);
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(REGISTER_DISTANCE);
  Wire.endTransmission(false);
  Wire.requestFrom(LIDAR_ADDR, 2);

  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    return (highByte << 8) + lowByte;
  } else {
    return -1;
  }
}
