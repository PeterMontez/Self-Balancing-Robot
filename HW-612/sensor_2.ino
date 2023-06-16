#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

float gX, gx2, aX, ax2;
MPU9250_asukiaaa mySensor;
Adafruit_BMP280 bme;

void setup() {
  Serial.begin(9600);
  setupSensor();
}

void loop() {
  Serial.println(String(getGyro()));
  //  Serial.println(String(getAccel()));
}
