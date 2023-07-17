
#include <MPU6050_6Axis_MotionApps20.h>


#include<SoftwareSerial.h>
SoftwareSerial car (50,52) ;
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
char control ;
float Angle ;
float InputAngle ;
float InitialAngle ;
float FinalAngle ;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



float GetAngle ()
{
  if (!dmpReady) return;
        while (!mpuInterrupt && fifoCount < packetSize) 
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
        {
           mpu.resetFIFO();
        } 
        else if (mpuIntStatus & 0x02) 
        {
           while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
           mpu.getFIFOBytes(fifoBuffer, packetSize);
           fifoCount -= packetSize;

         
           mpu.dmpGetGravity(&gravity, &q);
           mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
           Angle = ypr[0] * 180/M_PI + 180 ;
        }
        return Angle ;
}


void setup() 
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Serial.begin(9600) ;
  car.begin(9600) ;
  mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }   
  pinMode ( 3 , INPUT ) ;
}

void loop() {
 
   if (car.available()==1)
 {   
  control=car.read();
  Serial.println( control);  
    
}  
delay(25);
}
