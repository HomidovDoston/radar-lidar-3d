// Barcha kerakli kutubxonalarni chaqirish
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_MLX90614.h>

#define LIDAR_ADDR 0x62
#define BUZZER_PIN 9

MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  mlx.begin();
  pinMode(BUZZER_PIN, OUTPUT);
}

int readDistance() {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00);
  Wire.write(0x04);
  Wire.endTransmission();
  delay(20);

  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x8f);
  Wire.endTransmission();

  Wire.requestFrom(LIDAR_ADDR, 2);
  int high = Wire.read();
  int low = Wire.read();
  return (high << 8) + low;
}

void playBeep() {
  tone(BUZZER_PIN, 1000);
  delay(500);
  noTone(BUZZER_PIN);
}

void loop() {
  // Sensorlardan ma'lumot olish
  int distance = readDistance();
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float temp = mlx.readObjectTempC();

  // Shaklni aniqlash logikasi (soddalashtirilgan)
  String shape = (distance < 100 && temp > 30.0) ? "Silindr" : "Noma’lum";

  // Serial orqali Python'ga uzatish
  Serial.print("DIST:");
  Serial.print(distance);
  Serial.print(",TEMP:");
  Serial.print(temp);
  Serial.print(",GYRO:");
  Serial.print(gz); // faqat Z o‘qi
  Serial.print(",SHAPE:");
  Serial.println(shape);

  // Ogohlantirish
  if (distance < 150 && temp > 30.0) {
    playBeep();
  }

  delay(300);
}
