#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

void setupSensor(){

  Adafruit_BMP280 bme; // I2C
  MPU9250_asukiaaa mySensor;
  
  Wire.begin();
  mySensor.setWire(&Wire);

  bme.begin();
  mySensor.beginAccel();
}

int getAccel(){
  
}
