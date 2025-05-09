#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(9600);
  mlx.begin();
}

void loop() {
  float temp = mlx.readObjectTempC();
  Serial.print("Object Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");

  if (temp > 30.0) {
    Serial.println("Ehtimol odam aniqlandi.");
  }

  delay(1000);
}
