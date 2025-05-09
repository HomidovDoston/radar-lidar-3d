#include <Wire.h>
#define LIDAR_ADDR 0x62

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

int readDistance() {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00); // Reg: control
  Wire.write(0x04); // Take distance measurement
  Wire.endTransmission();

  delay(20);

  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x8f); // Reg: distance high byte
  Wire.endTransmission();

  Wire.requestFrom(LIDAR_ADDR, 2);
  int high = Wire.read();
  int low = Wire.read();
  return (high << 8) + low;
}

void loop() {
  int distance = readDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
}
