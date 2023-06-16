#include <MPU9250_asukiaaa.h>

float gX, gx2,aX, ax2;
MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(9600);
  Serial.println("Porta serial ligada!");
  setupSensor();
}

void loop() {
  Serial.println(String(getGyro()));
  Serial.println(String(getAccel()));
}
