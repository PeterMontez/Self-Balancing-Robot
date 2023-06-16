#include <MPU9250_asukiaaa.h>
//#include <Adafruit_BMP280.h>
#include <PID_v1.h>

#include "MPU9250.h"

#define MPU9250_IMU_ADDRESS 0x68

#define MAGNETIC_DECLINATION 1.63 // To be defined by user
#define INTERVAL_MS_PRINT 1000

MPU9250 mpu;

unsigned long lastPrintMillis = 0;

double setpoint= 180;

double Kp = 2; //Set this first
double Kd = 0.05; //Set this second
double Ki = 0; //Finally set this 

double input;
double output;

float aX;
float ax2;
float gX;
float gx2;

float valor;

int count = 0;

//MPU9250_asukiaaa mySensor;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(20);
  pid.SetOutputLimits(-255, 255);

  Wire.begin();
//  mySensor.setWire(&Wire);
//  mySensor.beginAccel();
  

  pinMode (6, OUTPUT);
  pinMode (9, OUTPUT);
  pinMode (10, OUTPUT);
  pinMode (11, OUTPUT);

  analogWrite(6,LOW);
  analogWrite(9,LOW);
  analogWrite(10,LOW);
  analogWrite(11,LOW);


  MPU9250Setting setting;

  // Sample rate must be at least 2x DLPF rate
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  mpu.setup(MPU9250_IMU_ADDRESS, setting);

  mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(15);

  Serial.println("Calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  delay(5000);

  Serial.println("Calibrating...");
  mpu.calibrateAccelGyro();

  Serial.println("Magnetometer calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);

  Serial.println("Calibrating...");
  mpu.calibrateMag();

  Serial.println("Ready!");
  
}

void loop() {

  unsigned long currentMillis = millis();

  if (mpu.update() && currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) {
    Serial.print("TEMP:\t");
    Serial.print(mpu.getTemperature(), 2);
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.print("C");
    Serial.println();

    Serial.print("Pitch:\t");
    Serial.print(mpu.getPitch());
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.println();

    Serial.print("Roll:\t");
    Serial.print(mpu.getRoll());
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.println();

    Serial.print("Yaw:\t");
    Serial.print(mpu.getYaw());
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.println();

    Serial.println();

    lastPrintMillis = currentMillis;
  }

  

  //Forward();
  //delay(5000);

//  int count = 0;
//
//  while(count < 5)
//  {
//    if (mySensor.accelUpdate() == 0) {
//      aX += mySensor.gyroX();
//      count++;
//    }
//  }
//
//  aX = aX/5;
  
  //valor = getGyro();
  
  input =  ((mpu.getYaw() * 180/M_PI) + 180);

  //aX = (mySensor.accelX() + mySensor.accelX() + mySensor.accelX() + mySensor.accelX() + mySensor.accelX()) / 5 * 100;

  Serial.print(mpu.getYaw());
  Serial.print(",");
  Serial.print(aX);
  Serial.print(",");
  //Serial.println(mySensor.accelX());

  double plsWork = 0;

  pid.Compute();
  plsWork += output;
  pid.Compute();
  plsWork += output;
  pid.Compute();
  plsWork += output;

  output = output/3;
  
  
  Serial.print(output);
  Serial.print(",");

  if (input>150 && input<200){
    if (output>0){
      Forward();
    }
  
    else if (output<0){
      Reverse();
    }
  }
  
  else{
    Stop();
  }
  
  
  Serial.println(input);
  
}

//mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
//mpu.dmpGetGravity(&gravity, &q); //get value for gravity
//mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr


void Reverse() //Code to rotate the wheel forward 
{
  analogWrite(6,0);
  analogWrite(9,output);
  analogWrite(10,output);
  analogWrite(11,0);
  
  //Serial.print("F"); //Debugging information 
}

void Forward() //Code to rotate the wheel Backward  
{
  analogWrite(6,output*-1);
  analogWrite(9,0);
  analogWrite(10,0);
  analogWrite(11,output); 

//Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
  analogWrite(6,0);
  analogWrite(9,0);
  analogWrite(10,0);
  analogWrite(11,0); 

//Serial.print("S");
}
