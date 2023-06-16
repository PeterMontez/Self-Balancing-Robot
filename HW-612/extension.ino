#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
void setupSensor(){
  
  Wire.begin();
  mySensor.setWire(&Wire);
  bme.begin();
  mySensor.beginAccel();
  mySensor.beginGyro();
}

float getGyro(){
  if(mySensor.accelUpdate() == 0){
    for(int i=0; i<5;i++){
      gX = mySensor.accelX();
      gx2 += gX * 100;
    }  
  }
  return (gx2 /= 5);
}

int getAccel(){
  if(mySensor.gyroUpdate() == 0){
    for(int i=0; i<5;i++){
      aX = mySensor.gyroX();
      ax2 += aX * 100;
    }  
  }
  return (ax2 /= 5);
}
