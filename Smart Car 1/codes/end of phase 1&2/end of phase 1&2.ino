#include <SoftwareSerial.h>
SoftwareSerial car(10,11);
int motor1_left=30 , motor1_right=32 , motor2_left=34 , motor2_right=36 ;
int pwm1 = 2 , pwm2 = 3 , key = 12 , left_sen = 40 ,right_sen = 42 ;
const byte trig = 6 , echo = 7 ;
double peroid , distance ;
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
 pinMode(motor2_right,INPUT);
 pinMode(motor2_right,INPUT);
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
  Serial.write(control);
   if(control=='s')
  {   
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);     
  } 
  else if(control=='a')
  {
    while (1)
    {
   if (distance > 40)
   {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
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
  else if(control=='b')
  {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
  else if(control=='m')
  {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
  else if(control=='n')
  {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);   
  }
  else if(control=='o')
  {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);   
  }
  else if(control=='p')
  {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
   else if(control=='c')
  {
    while (1)
    {
    if(distance > 40)
    {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
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
   else if(control=='d')
  {
    while (1)
    {
    if (distance > 40)
    {
    analogWrite(pwm1,68);
    analogWrite(pwm2,68);
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
 else if(control=='e')
  {
    while (1)
    {
    if (distance > 40)
    {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
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
  else if(control=='f')
  {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);
  }
  else if(control=='q')
  {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
  else if(control=='r')
  {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);   
  }
  else if(control=='y')
  {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);   
  }
  else if(control=='t')
  {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);   
  }
   else if(control=='g')
  {
    while (1)
    {
    if (distance > 40)
    {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);
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
   else if(control=='h')
  {
    while (1)
    {
    if (distance > 40)
    {
    analogWrite(pwm1,128);
    analogWrite(pwm2,128);    
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
  else if(control=='i')
  {
    while (1)
    {
    if (distance > 40)
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
    if (distance > 40)
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
    if (distance > 40 )
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
          analogWrite(pwm1,80);
          analogWrite(pwm2,80);
          digitalWrite(motor1_left,HIGH);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);
          }
          else if (digitalRead(left_sen)==LOW && digitalRead(right_sen)==HIGH)
          {
          analogWrite(pwm1,120);
          analogWrite(pwm2,120);
          digitalWrite(motor1_left,LOW);
          digitalWrite(motor1_right,HIGH);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);            
          }
          else if (digitalRead(left_sen)==HIGH && digitalRead(right_sen)==LOW)
          {
          analogWrite(pwm1,120);
          analogWrite(pwm2,120);
          digitalWrite(motor1_left,HIGH);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,LOW);
          digitalWrite(motor2_right,HIGH);         
          }
          else if (digitalRead(left_sen)==HIGH && digitalRead(right_sen)==HIGH)
          {
          analogWrite(pwm1,0);
          analogWrite(pwm2,0);
          digitalWrite(motor1_left,LOW);
          digitalWrite(motor1_right,LOW);
          digitalWrite(motor2_left,HIGH);
          digitalWrite(motor2_right,LOW);             
          }
          if(car.available()==1)
          break;  
      }                  
      }
  }     
 delay(20);
}
