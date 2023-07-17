#include <SoftwareSerial.h>
SoftwareSerial car(10,11);

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
int pwm1 = 4 , pwm2 = 5 , key = 12 ,dis_sen = 3 , left_sen = 40 ,right_sen = 42 ;
const byte trig = 6 , echo = 7 ;
double peroid , distance  ;

char control ;
char InputDigit ;

float InputDistance ;
String InputString ;
float InputFloat ;
int counter ;
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

void docount()  
{
  counter++;  
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
  
 Serial.begin(9600);
 car.begin(9600);
 pinMode(trig,OUTPUT);
 pinMode(echo,INPUT);
 pinMode(motor1_left,OUTPUT);
 pinMode(motor1_right,OUTPUT);
 pinMode(motor2_left,OUTPUT);
 pinMode(motor2_right,OUTPUT);
 pinMode(right_sen,INPUT);
 pinMode(left_sen,INPUT);
 pinMode(dis_sen,INPUT);
 pinMode(LED_PIN, OUTPUT);

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
 
}
 
void loop() {
  if (car.available()==1)
 {   
  control=car.read();
  
   if(control=='s')
   {   
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);     
  } 
   
  
  else if(control=='I')
  {
      for(int i=0 ; i<100 ; i++)
        {
          Angle = GetAngle() ;
          Serial.println(Angle) ;
          if ( car.available() == 1 )
          break ;      
        }
       InitialAngle = Angle ; 
       FinalAngle = Angle+180-5 ;
       while (FinalAngle > 360)
            {
              FinalAngle -= 360;
            }
     while (1)
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,90);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    Angle=GetAngle();    
    if(Angle>180 && FinalAngle<180)
    {
    Angle -=360;
    }
    Serial.println(Angle);
    if (Angle>=FinalAngle)
    {
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    delay(100);
    break;
    }
    }
    while (1)
    {
    analogWrite(pwm1,90);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);

    if(Angle<180 && InitialAngle>180)
    {
    Angle +=180;
    InitialAngle += 180 ;
    }
     if (Angle>=InitialAngle)
    {
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    delay(100);
    break;
    }
    }
  }  
  }
  delay(25);
}
