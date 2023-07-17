#include <SoftwareSerial.h>
SoftwareSerial car(10,11);
char control ;

#include <MPU6050_6Axis_MotionApps20.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int motor1_left=30 , motor1_right=32 , motor2_left=34 , motor2_right=36 ;
int pwm1 = 4 , pwm2 = 5 , key = 12  ;


char InputDigit ;
String InputString ;
float InputFloat ;
float Angle ;
float InputAngle ;
float InitialAngle ;
float FinalAngle ;
#define LED_PIN 13
bool blinkState = false;


Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;

}
float GetInput ()
{
  InputString = "" ;
  while ( 1 )
  {
    if ( car.available() == 1 )
    {
      InputDigit = car.read() ;
      if ( InputDigit != 'E' )
      {
        InputString += String(InputDigit) ;
      }
      else
      {
        break ;
      }
    }
  }
  InputFloat = atof ( InputString.c_str() ) ;
  return InputFloat ;
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
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
         mpu.getFIFOBytes(fifoBuffer, packetSize);    
        fifoCount -= packetSize;

       
            mpu.dmpGetQuaternion(&q, fifoBuffer);            
            
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            Angle = ypr[0] * 180/M_PI + 180 ;

            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
           
        }
        return Angle ;        
}
void setup() {
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
   Serial.begin(9600) ;
   car.begin(9600);
   pinMode(motor1_left,OUTPUT);
   pinMode(motor1_right,OUTPUT);
   pinMode(motor2_left,OUTPUT);
   pinMode(motor2_right,OUTPUT);

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
 pinMode(LED_PIN, OUTPUT);
}

void loop() {
   if (car.available()==1)
 {   
  control=car.read();
  Serial.println (control);
  
if ( control == 'A' )
    {
      
     for(int i=0 ; i<500 ; i++)
      {
        Angle = GetAngle() ;
        Serial.println(Angle) ;
        if ( car.available() == 1 )
        break ;
        
      }
      Serial.print("Angle is ") ;
      Serial.println(Angle) ;
      digitalWrite(LED_PIN, HIGH);
      
      
      InputAngle = GetInput() ;
      Serial.print("InputAngle is ") ;
      Serial.println(InputAngle) ;
       while ( InputAngle > 360 )
      {
        InputAngle -= 360 ;
      }
      while ( InputAngle < -360 )
      {
        InputAngle += 360 ;
      }
         InitialAngle = Angle ;
      FinalAngle = InitialAngle + InputAngle ;
      while ( FinalAngle > 360 )
      {
        FinalAngle -= 360 ;
      }
      while ( FinalAngle < -360 )
      {
        FinalAngle += 360 ;
      }
      Serial.print("FinalAngle is ") ;
      Serial.println(FinalAngle) ;
      while (1)
      {
     if ( InputAngle > 0 )
      {
        analogWrite(pwm1,100);
        analogWrite(pwm2,100);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
        if(Angle>=FinalAngle)  
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            break;
          }
      }
      else if ( InputAngle < 0 )
     {
        analogWrite(pwm1,100);
        analogWrite(pwm2,100);
        digitalWrite(motor1_left,LOW);
        digitalWrite(motor1_right,HIGH);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);  
         Angle = GetAngle();
         
        if(Angle<=FinalAngle)  
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            break;
          } 
      }
      }
  }
   else if(control=='s')
  {   
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);     
  }
 
  delay(50);
}
