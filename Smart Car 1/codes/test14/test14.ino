#include <SoftwareSerial.h>
SoftwareSerial car(10,11);
int motor1_left=30 , motor1_right=32 , motor2_left=34 , motor2_right=36 ;
int  key =12 ;
char control;

void setup() {
  Serial.begin(9600);
 car.begin(9600);
 pinMode(motor1_left,OUTPUT);
 pinMode(motor1_right,OUTPUT);
 pinMode(motor2_left,OUTPUT);
 pinMode(motor2_right,OUTPUT);
 pinMode(key,OUTPUT);
  
}
void loop() {
   digitalWrite(key,HIGH);
 if (car.available()==1) 
 {
   control=car.read();
  Serial.write(control);
  if(control=='a')
    {
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);  
    }
 else if(control=='b')
  {
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,HIGH);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,HIGH);
   
  }
   else if(control=='c')
  {   
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
   
  }
   else if(control=='d')
  {    
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    
  }  
  }
  delay (200);  
}
