#include <SoftwareSerial.h>
SoftwareSerial car(10,11);
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


int motor1_left=30 , motor1_right=32 , motor2_left=34 , motor2_right=36 ;
int pwm1 = 4 , pwm2 = 5 , key = 12 , left_sen = 40 ,right_sen = 42 ;
const byte trig = 6 , echo = 7 ;
double peroid , distance  ;
 double  d ;
char control ;
char InputDigit ;

int counter ;
int dis_sen = 22 ;

float Angle ; 
float InputAngle ;
float InitialAngle ;
float FinalAngle ;
float InputDistance ;
String InputString ;
float InputFloat ;
float Input ;

Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;    
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3]; 
        
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;  
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
     car.begin(9600);
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
    
    pinMode(trig,OUTPUT);
 pinMode(echo,INPUT);
 pinMode(motor1_left,OUTPUT);
 pinMode(motor1_right,OUTPUT);
 pinMode(motor2_left,OUTPUT);
 pinMode(motor2_right,OUTPUT);
 pinMode(right_sen,INPUT);
 pinMode(left_sen,INPUT);
 pinMode ( 4 , INPUT ) ;

}

void loop() {
  
  digitalWrite(key,HIGH);
  digitalWrite(trig,LOW);
  delayMicroseconds(3);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  peroid = pulseIn (echo,HIGH);
  distance = peroid/58.8;


   if (car.available()==1)
 {   
  control=car.read();
  Serial.println( control);  
  Serial.println( "\t"); 
  
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
    digitalWrite(motor2_right,LOW);
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
    digitalWrite(motor1_right,LOW);
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
    }

 else if (control=='z')
    {
      while (1)
      {
        if(digitalRead(left_sen)==LOW && digitalRead(right_sen)==LOW)
        {
          analogWrite(pwm1,90);
          analogWrite(pwm2,90);
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
          delay(5);
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
          delay(5);
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
          analogWrite(pwm1,90);
          analogWrite(pwm2,90);
          digitalWrite(motor1_left,HIGH);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);             
          }
          if(car.available()==1)
          break;  
      }                  
      }


else if ( control == 'A' )
    {
      while ( 1 )
      {
        Angle = GetAngle() ;
        //Serial.println(Angle) ;
        if ( car.available() == 1 )
           break ;           
      }      
      InputAngle = GetInput() ;
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
      while (1)
      {
 if ( InputAngle > 0 )
      {
        analogWrite(pwm1,80);
        analogWrite(pwm2,80);
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
        analogWrite(pwm1,80);
        analogWrite(pwm2,80);
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

else if (control == 'D')
{
  InputDistance = GetInput() ;
  counter=0;
  while(1)
  {
    if(dis_sen==LOW)
    counter++;
    if (counter >= (InputDistance/2.5))
    break;
    else
    {
    analogWrite(pwm1,155);
    analogWrite(pwm2,155);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
      }
    }
  
 }

 else if (control == 'S' )
 {
 counter=0;
    while(1)    
    {
    if(dis_sen==HIGH)
    counter++;
    if(counter>=40)
    {
    analogWrite(pwm1,155);
    analogWrite(pwm2,155);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);  
    }    
    else
     break;
  }
      Angle=GetAngle();
      FinalAngle=Angle+90;
      while (1)
      {
      if ( Angle <= FinalAngle )
      {
        analogWrite(pwm1,80);
        analogWrite(pwm2,80);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
      }
        else 
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            break;
          }
      }
     
    for (int i=1 ; i<=3 ; i++)
    {
   counter=0;
  while(1)
  {
    if(dis_sen==HIGH)
    counter++;
    if (counter < 80)
    {
    analogWrite(pwm1,155);
    analogWrite(pwm2,155);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
      }
      else
      {
        digitalWrite(motor1_left,LOW);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,LOW);
        break;
        }
  }
       Angle=GetAngle();
      FinalAngle=Angle+90;
      while (1)
      {
      if ( Angle <= FinalAngle )
      {
        analogWrite(pwm1,80);
        analogWrite(pwm2,80);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
      }
        else 
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            break;
          }
      }
}
counter=0;
    while(1)    
    {
    if(dis_sen==HIGH)
    counter++;
    if(counter>=40)
    {
    analogWrite(pwm1,155);
    analogWrite(pwm2,155);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);  
    }    
    else
     break;
  }

  
} 

}
delay(25);
}
