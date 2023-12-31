#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  
#include <I2Cdev.h>
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 
bool blinkState = false;


bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64];


Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;    
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3]; 
float Angle ;         
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;  
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

          #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
              #endif

        #ifdef OUTPUT_READABLE_EULER
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
             #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
             #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
             #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
           mpu.dmpGetQuaternion(&q, fifoBuffer);
           mpu.dmpGetAccel(&aa, fifoBuffer);
           mpu.dmpGetGravity(&gravity, &q);
           mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
           mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            #endif
    
        #ifdef OUTPUT_TEAPOT
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            teapotPacket[11]++; 
        #endif
         Angle = ypr[0] * 180/M_PI + 180 ;

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
            
        }
        return Angle ;
}
void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); 
    while (!Serial.available());                 
    while (Serial.available() && Serial.read()); 

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
  

}

void loop() {
 
 Angle=GetAngle();
 Serial.println(Angle);

}
