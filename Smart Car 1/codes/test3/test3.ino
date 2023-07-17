#include <SoftwareSerial.h>
SoftwareSerial project (12,13);
int key=11;
int red=5 , yellow=6 , blue=7 ;
char data ;

void setup() {
  Serial.begin(9600);
  Serial.println("entre data");
  project.begin(9600);
  pinMode(red,OUTPUT);
  pinMode(yellow,OUTPUT);
  pinMode(blue,OUTPUT);
  
}

void loop() {
  digitalWrite(key,HIGH);
  if(project.available()==1)
  {
    data=project.read();
    Serial.println(data);
    if(data=='r')
    {
      digitalWrite(red,HIGH);
      digitalWrite(yellow,LOW);
      digitalWrite(blue,LOW);
      project.println("red on");    
    }
   else if(data=='y')
    {
      digitalWrite(yellow,HIGH);
      digitalWrite(red,LOW);
      digitalWrite(blue,LOW); 
      project.println("yellow on");   
    }
    else if(data=='b')
    {
      digitalWrite(blue,HIGH);
      digitalWrite(yellow,LOW);
      digitalWrite(red,LOW); 
      project.println("blue on");   
    }
   else if(data=='a')
    {
      digitalWrite(red,HIGH);
      digitalWrite(yellow,HIGH);
      digitalWrite(blue,HIGH);  
      project.println("all led on");  
    }
    if(data=='s')
    {
      digitalWrite(red,LOW);
      digitalWrite(yellow,LOW);
      digitalWrite(blue,LOW); 
      project.println("all led off");   
    }
    else if(data=='m')
    {
      digitalWrite(red,HIGH);
      digitalWrite(yellow,LOW);
      digitalWrite(blue,HIGH);    
      project.println("red & blue on");
    }
    else if(data=='n')
    {
      digitalWrite(red,HIGH);
      digitalWrite(yellow,HIGH);
      digitalWrite(blue,LOW);  
      project.println("red & yellow on");  
    }
   else if(data=='o')
    {
      digitalWrite(red,LOW);
      digitalWrite(yellow,HIGH);
      digitalWrite(blue,HIGH);  
      project.println("yellow & blue on");  
    }
    
}
delay(200);
}
