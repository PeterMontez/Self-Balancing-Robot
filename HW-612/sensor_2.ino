#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

float aX, ax2;

void setup() {
  Serial.begin(9600);

  Serial.println("Porta serial ligada!");

  setupSensor();
  
  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

int count;
void loop() {
  Serial.println(String(getAccel()));
}
