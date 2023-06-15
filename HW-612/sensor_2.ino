#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa mySensor;
float aX, ax2;

void setup() {
  Serial.begin(9600);

  Serial.println("Porta serial ligada!");

  Wire.begin();
  mySensor.setWire(&Wire);

  bme.begin();
  mySensor.beginAccel();
  
  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

int count;
void loop() {
  if(count < 5){
    if (mySensor.accelUpdate() == 0)
    {
      aX = mySensor.accelX();
      ax2 += aX * 100;
//      Serial.print(" AclXXX: " + String(aX * 100));
//      Serial.println("");   
    }  
  }
  else{
    ax2 /= count;
    Serial.println("Mean acl: " + String(ax2));
    count = 0;
  }
  
  count++;
  
}
