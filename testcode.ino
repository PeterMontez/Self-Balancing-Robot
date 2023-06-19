/*Arduino Self Balancing Robot

 * Code by: B.Aswinth Raj

 * Build on top of Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

 * Website: circuitdigest.com 

 */



#include "I2Cdev.h"

#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h

#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050


MPU6050 mpu;


// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

uint16_t fifoCount;     // count of all bytes currently in FIFO

uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars

Quaternion q;           // [w, x, y, z]         quaternion container

VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


 


/*********Tune these 4 values for your BOT*********/

double setpoint= 179; //set the value when the bot is perpendicular to ground using //////Serial monitor. 

//Read the project documentation on circuitdigest.com to learn how to set these values

double Kp = 14; //Set this first

double Kd = 0.005; //Set this secound

double Ki = 9; //Finally set this 

/******End of values setting*********/



double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


 



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady()

{

    mpuInterrupt = true;

}



void setup() {

  ////////Serial.begin(115200);


  // initialize device


    mpu.initialize();


     // verify connection


    // load and configure the DMP

    while(mpu.dmpInitialize() != 0){
      mpu.initialize();
       devStatus = mpu.dmpInitialize();
    }

    devStatus = mpu.dmpInitialize();


    

    // supply your own gyro offsets here, scaled for min sensitivity

    mpu.setXGyroOffset(220);

    mpu.setYGyroOffset(76);

    mpu.setZGyroOffset(-85);

    mpu.setZAccelOffset(1688); 


      // make sure it worked (returns 0 if so)

    if (devStatus == 0)

    {

        // turn on the DMP, now that it's ready


        mpu.setDMPEnabled(true);


        // enable Arduino interrupt detection


        attachInterrupt(0, dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus();


        // set our DMP Ready flag so the main loop() function knows it's okay to use it


        dmpReady = true;


        // get expected DMP packet size for later comparison

        packetSize = mpu.dmpGetFIFOPacketSize();

        

        //setup PID

        pid.SetMode(AUTOMATIC);

        pid.SetSampleTime(15);

        pid.SetOutputLimits(-255, 255);  

    }

    else

    {

        // ERROR!

        // 1 = initial memory load failed

        // 2 = DMP configuration updates failed

        // (if it's going to break, usually the code will be 1)


    }


//Initialise the Motor outpu pins

    pinMode (6, OUTPUT);

    pinMode (9, OUTPUT);

    pinMode (10, OUTPUT);

    pinMode (11, OUTPUT);


//By default turn off both the motors

    analogWrite(6,LOW);

    analogWrite(9,LOW);

    analogWrite(10,LOW);

    analogWrite(11,LOW);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);

}


 


void loop() {

    

    // if programming failed, don't try to do anything

    if (!dmpReady) {
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
      return;
    }


    // wait for MPU interrupt or extra packet(s) available

    while (!mpuInterrupt && fifoCount < packetSize)

    {

        //no mpu data - performing PID calculations and output to motors     

        pid.Compute();   

        

        //Print the value of Input and Output on //////Serial monitor to check how it is working.

        //////Serial.print(input);
        //////Serial.print(",");
        //////Serial.println(output);

               

        if (input>150 && output < 210){//If the Bot is falling 
          if (output>0) {//Falling towards front 
            Forward(); //Rotate the wheels forward 
          }
  
          else if (output<0) {//Falling towards back
            Reverse(); //Rotate the wheels backward 
          }
        
        }

        else {//If Bot not falling

          Stop(); //Hold the wheels still
        }
        

    }


    // reset interrupt flag and get INT_STATUS byte

    mpuInterrupt = false;

    mpuIntStatus = mpu.getIntStatus();


    // get current FIFO count

    fifoCount = mpu.getFIFOCount();


    // check for overflow (this should never happen unless our code is too inefficient)

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)

    {

        // reset so we can continue cleanly

        mpu.resetFIFO();



    // otherwise, check for DMP data ready interrupt (this should happen frequently)

    }

    else if (mpuIntStatus & 0x02)

    {

        // wait for correct available data length, should be a VERY short wait

        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        // read a packet from FIFO

        mpu.getFIFOBytes(fifoBuffer, packetSize);

        

        // track FIFO count here in case there is > 1 packet available

        // (this lets us immediately read more without waiting for an interrupt)

        fifoCount -= packetSize;


        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q

        mpu.dmpGetGravity(&gravity, &q); //get value for gravity

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr


        input = ypr[1] * 180/M_PI + 180;


   }

}


void Reverse() //Code to rotate the wheel forward 

{
    analogWrite(13, output*-1);

    analogWrite(6,output*-1);

    analogWrite(9,0);

    analogWrite(10,0);

    analogWrite(11,output*-1);

    //////Serial.print("REVERSE: ");
    //////Serial.println(output*-1);

}


void Forward() //Code to rotate the wheel Backward  

{

    analogWrite(6,0);

    analogWrite(9,output);

    ////Serial.print("FORWARD: ");
    ////Serial.println(output);

    analogWrite(10,output);

    analogWrite(11,0); 

}


void Stop() //Code to stop both the wheels

{

    analogWrite(6,0);

    analogWrite(9,0);

    analogWrite(10,0);

    analogWrite(11,0); 

}
