#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>

double setpoint= 180;

double Kp = 21; //Set this first
double Kd = 0.8; //Set this secound
double Ki = 140; //Finally set this 

double input, output;

float aX, ax2;

int count = 0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  pinMode (6, OUTPUT);
  pinMode (9, OUTPUT);
  pinMode (10, OUTPUT);
  pinMode (11, OUTPUT);

  analogWrite(6,LOW);
  analogWrite(9,LOW);
  analogWrite(10,LOW);
  analogWrite(11,LOW);
}

void loop() {
  if(count < 5){
    if (mySensor.accelUpdate() == 0)
    {
      aX = mySensor.accelX();
      ax2 += aX * 100;
    }  
  }
  else{
    ax2 /= count;
    Serial.println("Mean acl: " + String(ax2));
    count = 0;
  }
  
  count++;

  pid.Compute();
  
  Serial.print(input); Serial.print(" =>"); Serial.println(output);

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
}

//mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
//mpu.dmpGetGravity(&gravity, &q); //get value for gravity
//mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

input = ypr[1] * 180/M_PI + 180;
}
}

void Forward() //Code to rotate the wheel forward 
{
analogWrite(6,output);
analogWrite(9,0);
analogWrite(10,output);
analogWrite(11,0);

Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
analogWrite(6,0);
analogWrite(9,output*-1);
analogWrite(10,0);
analogWrite(11,output*-1); 

Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
analogWrite(6,0);
analogWrite(9,0);
analogWrite(10,0);
analogWrite(11,0); 

Serial.print("S");
}
