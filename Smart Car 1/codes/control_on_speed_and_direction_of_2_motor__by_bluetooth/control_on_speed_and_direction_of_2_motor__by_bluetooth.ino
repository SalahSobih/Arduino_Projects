#include <SoftwareSerial.h> 
SoftwareSerial project (8,9);
int key=10;
int dir1_mor1=30 , dir2_mor1=32 , dir1_mor2=34 , dir2_mor2=36 ;
int pwm1=5 , pwm2=6 ;
char sp1 , sp2 ;
int  control ;


void setup() {
 Serial.begin(9600);
 Serial.println("entre data");
 project.begin(9600);
 pinMode(key,OUTPUT);
 pinMode(dir1_mor1,OUTPUT);
 pinMode(dir2_mor1,OUTPUT);
 pinMode(dir1_mor2,OUTPUT);
 pinMode(dir2_mor2,OUTPUT);
 
}

void loop() {
  digitalWrite(key,HIGH);
 if(project.available()==1)
 {
  sp1=project.read();
  sp2=project.read();
  control=project.read();
   Serial.println(sp1);
   Serial.println(sp2);
   Serial.println(control);
  if(control==1)
  {
    digitalWrite(dir1_mor1,HIGH);
    digitalWrite(dir1_mor2,HIGH);
    digitalWrite(dir2_mor1,LOW);
    digitalWrite(dir2_mor2,LOW);  
  }
else if(control==2)
  {
    digitalWrite(dir1_mor1,LOW);
    digitalWrite(dir1_mor2,LOW);
    digitalWrite(dir2_mor1,HIGH);
    digitalWrite(dir2_mor2,HIGH);  
  }
  else if(control==3)
  {
    digitalWrite(dir1_mor1,HIGH);
    digitalWrite(dir1_mor2,LOW);
    digitalWrite(dir2_mor1,LOW);
    digitalWrite(dir2_mor2,HIGH);  
  }
else if(control==4)
  {
    digitalWrite(dir1_mor1,LOW);
    digitalWrite(dir1_mor2,HIGH);
    digitalWrite(dir2_mor1,HIGH);
    digitalWrite(dir2_mor2,LOW);  
  }
  else if(control==5)
  {
    digitalWrite(dir1_mor1,HIGH);
    digitalWrite(dir1_mor2,LOW);
    digitalWrite(dir2_mor1,LOW);
    digitalWrite(dir2_mor2,LOW);  
  }
  else if(control==6)
  {
    digitalWrite(dir1_mor1,LOW);
    digitalWrite(dir1_mor2,LOW);
    digitalWrite(dir2_mor1,HIGH);
    digitalWrite(dir2_mor2,LOW);  
  }
  else if(control==7)
  {
    digitalWrite(dir1_mor1,LOW);
    digitalWrite(dir1_mor2,HIGH);
    digitalWrite(dir2_mor1,LOW);
    digitalWrite(dir2_mor2,LOW);  
  }
  else if(control==8)
  {
    digitalWrite(dir1_mor1,LOW);
    digitalWrite(dir1_mor2,LOW);
    digitalWrite(dir2_mor1,LOW);
    digitalWrite(dir2_mor2,HIGH);  
  }
  if(sp1=='l')
  {
    float l=64.0;
    analogWrite(pwm1,l); 
  }
  else if(sp1=='m')
  {
    float m=128.0;
    analogWrite(pwm1,m); 
  }
   else if(sp1=='h')
  {
    float h=255.0;
    analogWrite(pwm1,h); 
  }
   if(sp2=='L')
  {
   float L=64.0;
    analogWrite(pwm2,L); 
  }
   else if(sp2=='M')
  {
    float M=128.0;
    analogWrite(pwm2,M); 
  }
   else if(sp2=='H')
  {
    float H=255.0;
    analogWrite(pwm2,H); 
  }
 }
delay(200);
}
