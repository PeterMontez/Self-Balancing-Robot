#include "I2Cdev.h"
#include <PID_v1.h>                     // https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" // https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;    // Creates the mpu object

bool dmpReady = false; // Inicialize variable as false, to check later if the gyro initialized correctly

uint8_t mpuIntStatus;
uint8_t devStatus;      // not in use, but 0 means working, !0 means error
uint16_t packetSize;    // default DMP packet size (42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation / Moves
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// -----------  PID CONTROL / MOTOR ATENUATION  -----------

double setpoint = 179.6; // Angle in wich the robot stays perfectly ballanced

double Kp = 14; // Proporcional constant

double Kd = 0.65; // Derivative constant

double Ki = 100; // Integral constant

double atenuation = 10;  // Baseline power to the motors

// -----------  PID declaration  -----------

double input, output, move;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; // detects if MPU interrupt pin is high

// -----------  Interrupt Function  -----------

void dmpDataReady()
{
    mpuInterrupt = true;
}

// -----------  Main code Setup  -----------

void setup()
{
    Serial.begin(115200);

    mpu.initialize();   // initializes the sensor

    while (mpu.dmpInitialize() != 0)    // keeps trying to init the sensor until it works
    {
        mpu.initialize();
        devStatus = mpu.dmpInitialize();    // sets the status to not 0
    }

    devStatus = mpu.dmpInitialize();

    // -----------  Gyro offsets  -----------

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    // make sure everything is working

    if (devStatus == 0)
    {
        // turn DMP on
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set DMP flag to true
        dmpReady = true;

        // get mpu packetsize
        packetSize = mpu.dmpGetFIFOPacketSize();

        // setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(15);
        pid.SetOutputLimits(-255, 255);
    }

    // -----------  Motor pins setup  -----------

    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    // -----------  Making sure motors initialize on LOW  -----------

    analogWrite(6, LOW);
    analogWrite(9, LOW);
    analogWrite(10, LOW);
    analogWrite(11, LOW);

    // -----------  Does a 1sec flash of the onboard LED  -----------
    // -----------  to signal that setup ran properly  -----------

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
}

// -----------  Main Code Loop  -----------

void loop()
{
    if (!dmpReady)      // checks if dmp is working. If not, onboard LED blinks rapidly
    {
        digitalWrite(13, HIGH);
        delay(50);
        digitalWrite(13, LOW);
        delay(50);
        return;
    }

    while (!mpuInterrupt && fifoCount < packetSize)     // While there's no new data, run this
    {
        pid.Compute();      // Computes PID

        Atenuate();     // Fixes output values

        Serial.print(move);
        Serial.print(",");
        Serial.println(input);

        if (input > 150 && input < 210)    // Turns the motors if the robot is in a reasonable position to do so
        {

            if (output > 0)
            {
                Forward();
            }

            else if (output < 0)
            {
                Reverse();
            }
        }

        else
        {
            Stop();
        }
    }

    mpuInterrupt = false;   // sets interrupt flag to false
    mpuIntStatus = mpu.getIntStatus();  // gets interrupt status

    fifoCount = mpu.getFIFOCount();     // gets size of FIFO

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)     // If interrupt hits or fifo size maxes out, reset FIFO
    {
        mpu.resetFIFO();
    }

    else if (mpuIntStatus & 0x02)   
    {
        // wait until data length is the correct size
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        mpu.getFIFOBytes(fifoBuffer, packetSize);   // gets data
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);      // get value for q
        mpu.dmpGetGravity(&gravity, &q);           // get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // get value for ypr

        input = ypr[1] * 180 / M_PI + 180;      // Calculates the current angle of the robot, and sets it as the PID input
    }

}



void Atenuate()     // Atenuates the motor to always give it a baseline current
{
    if (output > 0) {
            move = output + atenuation;
            if (move > 255)
            {
                move = 255;
            }
        }
        else if (output < 0)
        {
            move = output - atenuation;
            if (move < -255)
            {
                move = -255;
            }
        }
}

void Reverse() // Rotates backward
{
    analogWrite(13, move * -1);
    analogWrite(6, move * -1);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, move * -1);
}
void Forward() // Rotates forward
{
    analogWrite(6, 0);
    analogWrite(9, move);
    analogWrite(10, move);
    analogWrite(11, 0);
}

void Stop() // Stop wheels
{
    analogWrite(6, 0);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
}
