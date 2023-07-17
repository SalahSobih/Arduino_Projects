
#include <SoftwareSerial.h>
SoftwareSerial car(10,11);
int motor1_left=30 , motor1_right=32 , motor2_left=34 , motor2_right=36 ;
int pwm1 = 4 , pwm2 = 5 , key = 12 , left_sen = 40 ,right_sen = 42 ;
const byte trig = 6 , echo = 7 ;
double peroid , distance  ;
double  d ;
char control ;


void setup() {
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
   Serial.println (distance);
  
   
 if (car.available()==1)
 {   
  control=car.read();
  Serial.println (control);
  
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
  }     
 delay(25);
}
