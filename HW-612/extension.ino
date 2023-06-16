#include <MPU9250_asukiaaa.h>

void setupSensor(){
  
  Wire.begin();
  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
}

float getGyro(){
  if(mySensor.accelUpdate() == 0){
    for(int i=0; i<5;i++){
      gX += (mySensor.accelX() * 100);
    }  
  }
  return (gX /= 5);
}

int getAccel(){
  if(mySensor.gyroUpdate() == 0){
    for(int i=0; i<5;i++){
      aX += (mySensor.gyroX() * 100);
    }  
  }
  return (aX /= 5);
}
