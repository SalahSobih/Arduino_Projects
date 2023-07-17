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
  digitalWrite(key,LOW);
  digitalWrite(trig,LOW);
  delayMicroseconds(3);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  peroid = pulseIn (echo,HIGH);
  distance = peroid/58.8;
  Serial.println(distance);
  
   
 if (car.available()==1)
 {   
  control=car.read();
  Serial.println(control);
  
   if(control=='s')
  {   
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);     
  } 
 

  else if(control=='i')
  {
    while (1)
    {
    if (distance > 15)
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    }
    else 
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);
    delay (500);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    break;
    }
    digitalWrite(trig,LOW);
    delayMicroseconds(3);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    peroid = pulseIn (echo,HIGH);
    distance = peroid/58.8;
    if(car.available()==1)
    break;
    }
  }
  else if(control=='j')
  {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);    
  }
  else if(control=='u')
  {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
  else if(control=='v')
  {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);   
  }
  else if(control=='w')
  {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);   
  }
  else if(control=='x')
  {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
   else if(control=='k')
  {
    while (1)
    {
    if (distance > 15)
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    }
    else 
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);    
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    delay (1000);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    break;
    }
    digitalWrite(trig,LOW);
    delayMicroseconds(3);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    peroid = pulseIn (echo,HIGH);
    distance = peroid/58.8;
    if(car.available()==1)
    break;
    }
  }
  
   else if(control=='l')
  {
    while (1)
    {
    if (distance > 15)
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);    
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    }
    else 
    {
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);
    delay (1000);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    break;
    }
    digitalWrite(trig,LOW);
    delayMicroseconds(3);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    peroid = pulseIn (echo,HIGH);
    distance = peroid/58.8;
    if(car.available()==1)
    break;
    }
  }
    else if (control=='z')
    {
      while (1)
      {
        if(digitalRead(left_sen)==LOW && digitalRead(right_sen)==LOW)
        {
          analogWrite(pwm1,100);
          analogWrite(pwm2,100);
          digitalWrite(motor1_left,HIGH);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);
          }
          else if (digitalRead(left_sen)==LOW && digitalRead(right_sen)==HIGH)
          {
          digitalWrite(motor1_left,LOW);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,LOW);
          digitalWrite(motor2_right,LOW);      
          delay(10);
          analogWrite(pwm1,200);
          analogWrite(pwm2,200);
          digitalWrite(motor1_left,LOW);
          digitalWrite(motor1_right,HIGH);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);
          delay(10);            
          }
          else if (digitalRead(left_sen)==HIGH && digitalRead(right_sen)==LOW)
          {
          digitalWrite(motor1_left,LOW);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,LOW);
          digitalWrite(motor2_right,LOW);      
          delay(10);
          analogWrite(pwm1,200);
          analogWrite(pwm2,200);
          digitalWrite(motor1_left,HIGH);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,LOW);
          digitalWrite(motor2_right,HIGH);   
          delay(10);      
          }
          else if (digitalRead(left_sen)==HIGH && digitalRead(right_sen)==HIGH)
          {
          analogWrite(pwm1,100);
          analogWrite(pwm2,100);
          digitalWrite(motor1_left,HIGH);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);             
          }
          if(car.available()==1)
          break;  
      }                  
      }
 else if (control =='D' ) 
  {
     InputDistance = GetInput() ; 
     float c = (InputDistance/1.45);
     if (InputDistance<0)
     c=-c ;
     counter =0 ;
     while (counter < c)
     {
     if (InputDistance>0)
     {
     analogWrite(pwm1,255);
     analogWrite(pwm2,255);
     digitalWrite(motor1_left,HIGH);
     digitalWrite(motor1_right,LOW);
     digitalWrite(motor2_left,HIGH);
     digitalWrite(motor2_right,LOW);
     }
     else 
     {
     analogWrite(pwm1,255);
     analogWrite(pwm2,255);
     digitalWrite(motor1_left,LOW);
     digitalWrite(motor1_right,HIGH);
     digitalWrite(motor2_left,LOW);
     digitalWrite(motor2_right,HIGH);
     }
     
    attachInterrupt ( digitalPinToInterrupt(3) , docount , CHANGE ) ;
    Serial.println(counter) ;
  } 
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW); 
    }
    
 else if ( control == 'A' )  
 {
   for(int i=0 ; i<500 ; i++)
      {
        Angle = GetAngle() ;
        Serial.println(Angle) ;
        if ( car.available() == 1 )
        break ;
        
      }
      //Serial.print("Angle is ") ;
     // Serial.println(Angle) ;
      digitalWrite(LED_PIN, HIGH);
      
      
      InputAngle = GetInput() ;
     // Serial.print("InputAngle is ") ;
     // Serial.println(InputAngle) ;
       while ( InputAngle > 360 )
      {
        InputAngle -= 360 ;
      }
      while ( InputAngle < -360 )
      {
        InputAngle += 360 ;
      }
         InitialAngle = Angle ;
      FinalAngle = InitialAngle + InputAngle -7 ;
      while ( FinalAngle > 360 )
      {
        FinalAngle -= 360 ;
      }
       while ( FinalAngle < 0 )
      {
        FinalAngle += 360 ;
      }
     
     // Serial.print("FinalAngle is ") ;
     // Serial.println(FinalAngle) ;
      while (1)
      {
     if ( InputAngle > 0 )
      {
        analogWrite(pwm1,150);
        analogWrite(pwm2,150);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
         if ( Angle > 270  && FinalAngle < 90 )
      {
        Angle -= 360 ;
      }
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
        analogWrite(pwm1,150);
        analogWrite(pwm2,150);
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
   else if(control=='S')
  {
     for(int i=0 ; i<500 ; i++)
      {
        Angle = GetAngle() ;
       Serial.println(Angle) ;
          if ( car.available() == 1 )
        break ;     
      }
     // Serial.print("Angle is ") ;
     //Serial.println(Angle) ;
    
      //Serial.print("final Angle is ") ;
      //Serial.println(FinalAngle) ;
     counter =0 ;
     while (counter < 40)
     {
     analogWrite(pwm1,187);
     analogWrite(pwm2,200);
     digitalWrite(motor1_left,HIGH);
     digitalWrite(motor1_right,LOW);
     digitalWrite(motor2_left,HIGH);
     digitalWrite(motor2_right,LOW);
   
    attachInterrupt ( digitalPinToInterrupt(3) , docount , CHANGE ) ;
   // Serial.println(counter) ;
     } 
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    delay(200);
    
      
      FinalAngle=Angle+90-5;
       while ( FinalAngle > 360 )
      {
        FinalAngle -= 360 ;
        Angle -= 360;
      }
     
     while (1)
      {
      if ( Angle <= FinalAngle )
      {
        analogWrite(pwm1,110);
        analogWrite(pwm2,110);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
        if ( Angle > 270 && FinalAngle < 90 )
      {
        Angle -= 360 ;
      }
      }
        else 
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            delay(200);
            break;
          }
      }
          
      for (int i=1 ; i<=3 ; i++)
    {
      
     Angle = GetAngle();    
     counter = 0 ;
     while (counter < 80)
     {
     analogWrite(pwm1,187);
     analogWrite(pwm2,200);
     digitalWrite(motor1_left,HIGH);
     digitalWrite(motor1_right,LOW);
     digitalWrite(motor2_left,HIGH);
     digitalWrite(motor2_right,LOW);
   
    attachInterrupt ( digitalPinToInterrupt(3) , docount , CHANGE ) ;
    Serial.println(counter) ;
     } 
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    delay(200);
    
     
      FinalAngle=Angle+90;
      
       
   while ( FinalAngle > 360 )
    {
      FinalAngle -= 360 ;
      Angle -= 360;
    }
          
     while (1)
      {
      if ( Angle <= FinalAngle )
      {
        analogWrite(pwm1,110);
        analogWrite(pwm2,110);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
         if ( Angle > 270  && FinalAngle < 90 )
      {
        Angle -= 360 ;
      }
      }
        else 
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW);
            delay(200); 
            break;
          }
           
      }
             
    }
    counter =0 ;
     while (counter < 30)
     {
     analogWrite(pwm1,187);
     analogWrite(pwm2,200);
     digitalWrite(motor1_left,HIGH);
     digitalWrite(motor1_right,LOW);
     digitalWrite(motor2_left,HIGH);
     digitalWrite(motor2_right,LOW);
   
    attachInterrupt ( digitalPinToInterrupt(3) , docount , CHANGE ) ;
    Serial.println(counter) ;
     } 
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);        
    }
     else if(control=='C')
  {
    
     for(int i=0 ; i<500 ; i++)
      {
        Angle = GetAngle() ;
        Serial.println(Angle) ;
        if ( car.available() == 1 )
        break ;      
      }
    FinalAngle = Angle ;
    analogWrite(pwm1,255);
    analogWrite(pwm2,110);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    Serial.print("Angle is ");
    Serial.println(Angle );
    Serial.print("FinalAngle is ");
    Serial.println(FinalAngle );
    delay (2000);
      for(int i=0 ; i<200 ; i++)
      {
        Angle = GetAngle() ;
        Serial.println(Angle) ;
        if ( car.available() == 1 )
        break ;      
      }
    while(1)
    {        
    analogWrite(pwm1,255);
    analogWrite(pwm2,110);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
   // Serial.print("Angle is ");
   // Serial.println(Angle );
    Serial.print("FinalAngle is ");
    Serial.println(FinalAngle );
    Angle =GetAngle ();
     Serial.println(Angle );
    if (Angle <= FinalAngle+1 && Angle >= FinalAngle-1 )
    {
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
    Serial.print("Angle* is ");
    Serial.println(Angle );
    Serial.print("FinalAngle* is ");
    Serial.println(FinalAngle );
    break; 
    }
     
    }
  }
   else if(control=='I')
  {
    while (1)
    {
     for(int i=0 ; i<500 ; i++)
      {
        Angle = GetAngle() ;
        Serial.println(Angle) ;
        if ( car.available() == 1 )
        break ;      
      }
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(250); 
        analogWrite(pwm1,255);
        analogWrite(pwm2,90);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(3500);
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(500);  
        analogWrite(pwm1,90);
        analogWrite(pwm2,255);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(3500);
        digitalWrite(motor1_left,LOW);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,LOW);
        delay(500);
        for(int i=0 ; i<500 ; i++)
      {
        Angle = GetAngle() ;
        Serial.println(Angle) ;
        if ( car.available() == 1 )
        break ;      
      }       
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(500); 
        analogWrite(pwm1,90);
        analogWrite(pwm2,255);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(3500); 
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(500); 
        analogWrite(pwm1,255);
        analogWrite(pwm2,90);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(3500);
        analogWrite(pwm1,200);
        analogWrite(pwm2,200);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);
        delay(250);
        digitalWrite(motor1_left,LOW);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,LOW);
        break; 
  }
  }
 else if (control == 'T') 
 {
  while (1)
  {
  control = 'S';
  delay (5000);
  control = 'C';
  delay (5000);
  control = 'I';
  break;
  }
  }    
 }     
 delay(25);
}
