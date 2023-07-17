#include <SoftwareSerial.h> 
SoftwareSerial project (12,13);
const int key=11;
const int other=3 , right=4 , back=5 ,forward=6 , left=7 ;
char control;

void setup() {
 Serial.begin(9600);
 Serial.println("entre data");
 project.begin(9600);
 pinMode(forward,OUTPUT);
 pinMode(back,OUTPUT);
 pinMode(right,OUTPUT);
 pinMode(left,OUTPUT);
 pinMode(other,OUTPUT);
}

void loop() {
  digitalWrite(key,HIGH);
 if(project.available()==1)
 {
  control=project.read();
  if(control=='r')
  {
     
    digitalWrite(forward,HIGH);
    digitalWrite(back,LOW);
    digitalWrite(right,LOW);
    digitalWrite(left,LOW); 
    digitalWrite(other,LOW);  
   }
 else if(control=='y')
  {
    digitalWrite(forward,LOW);
    digitalWrite(back,HIGH);
    digitalWrite(right,LOW);
    digitalWrite(left,LOW); 
    digitalWrite(other,LOW);     
   }  
    else if(control=='b')
  { 
    digitalWrite(forward,LOW);
    digitalWrite(back,LOW);
     digitalWrite(left,LOW); 
    digitalWrite(other,LOW);
   
     digitalWrite(right,HIGH); 
     delay(500);
     digitalWrite(right,LOW);
     delay(500);
     digitalWrite(right,HIGH); 
     delay(500);
     digitalWrite(right,LOW);
     delay(500);
     digitalWrite(right,HIGH); 
     delay(500);
     digitalWrite(right,LOW);
     delay(500);
     digitalWrite(right,HIGH); 
     delay(500);
     digitalWrite(right,LOW);
     delay(500);
     digitalWrite(right,HIGH); 
     delay(500);
     digitalWrite(right,LOW);
     delay(500);
  }
     else if(control=='l')
  {
    digitalWrite(forward,LOW);
    digitalWrite(back,LOW);
    digitalWrite(right,LOW); 
    digitalWrite(other,LOW);
   
     digitalWrite(left,HIGH); 
     delay (500);
     digitalWrite(left,LOW);
     delay (500);
      digitalWrite(left,HIGH); 
     delay (500);
     digitalWrite(left,LOW);
     delay (500);
      digitalWrite(left,HIGH); 
     delay (500);
     digitalWrite(left,LOW);
     delay (500);
      digitalWrite(left,HIGH); 
     delay (500);
     digitalWrite(left,LOW);
     delay (500);
      digitalWrite(left,HIGH); 
     delay (500);
     digitalWrite(left,LOW);
     delay (500);
     
  }
  else if(control=='m')
  { 
    digitalWrite(forward,LOW);
    digitalWrite(back,LOW);
    digitalWrite(right,LOW);
    digitalWrite(left,LOW); 
    digitalWrite(other,HIGH);  
  }
     
}
delay(100);
}
