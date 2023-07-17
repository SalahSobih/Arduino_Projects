#include <SoftwareSerial.h>
SoftwareSerial project (12,13);
int key=11;
int color=3 , blue1=4 , yellow=5 , red=6 , blue2=7 ;
char data ;

void setup() {
  Serial.begin(9600);
  Serial.println("entre data");
  project.begin(9600);
  pinMode(key,OUTPUT)
  pinMode(red,OUTPUT);
  pinMode(yellow,OUTPUT);
  pinMode(blue1,OUTPUT);
  pinMode(blue2,OUTPUT);
  pinMode(color,OUTPUT);
  
}

void loop() {
  digitalWrite(key,HIGH);
  if(project.available()==1)
  {
    digitalWrite(color,HIGH);
    data=project.read();
    if(data=='r')
    {
      digitalWrite(red,HIGH);
      digitalWrite(yellow,LOW);
      digitalWrite(blue1,LOW);
      digitalWrite(blue2,LOW);
          
    }
   else if(data=='y')
    {
      digitalWrite(yellow,HIGH);
      digitalWrite(red,LOW);
      digitalWrite(blue1,LOW); 
      digitalWrite(blue2,LOW);   
    }
    else if(data=='b')
    {      
      digitalWrite(yellow,LOW);
      digitalWrite(red,LOW); 
      digitalWrite(blue2,LOW);
      while (1)
      {
        digitalWrite(blue1,HIGH);
        delay(200);
        digitalWrite(blue1,LOW);
        delay(200);
        if(project.available()==1)
        break;
        
       }   
    }
   else if(data=='p')
    {
      digitalWrite(red,LOW);
      digitalWrite(yellow,LOW);        
      digitalWrite(blue1,LOW);  
      while (1)
      {
        digitalWrite(blue2,HIGH);
        delay(200);
        digitalWrite(blue2,LOW);
        delay(200);
        if (project.available()==1)
        break;
        
      }
    }
    
     else if(data=='s')
    {
      digitalWrite(red,LOW);
      digitalWrite(yellow,LOW);
      digitalWrite(blue2,LOW);  
      digitalWrite(blue1,LOW);  
    }
   
    
}
delay(200);
}
