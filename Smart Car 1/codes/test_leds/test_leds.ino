#include <SoftwareSerial.h>
SoftwareSerial leds (12,13);
int key=11;
int blue1=3 , red=4 , green=5 , blue2=6  ;
char data ;

void setup() {
  Serial.begin(9600);
  leds.begin(9600);
 pinMode(blue1,OUTPUT);
 pinMode(red,OUTPUT);
 pinMode(green,OUTPUT);
 pinMode(blue2,OUTPUT);
 pinMode(key,OUTPUT);
}

void loop() {
  digitalWrite(key,HIGH); 
  if (leds.available()==1)
  {
    data=leds.read();
    if (data=='r') 
    {
      digitalWrite(red,HIGH);
      digitalWrite(blue1,LOW);      
      digitalWrite(green,LOW);
      digitalWrite(blue2,LOW); 
     }
     else if (data=='g') 
    {
      digitalWrite(green,HIGH);
      digitalWrite(blue1,LOW);
      digitalWrite(red,LOW);      
      digitalWrite(blue2,LOW); 
     }
   else if (data=='l') 
    {
      digitalWrite(blue1,HIGH);
      digitalWrite(red,LOW);
      digitalWrite(green,LOW);
      digitalWrite(blue2,LOW);               
     }
      else if (data=='b') 
    {
      digitalWrite(blue2,HIGH);
      digitalWrite(red,LOW);
      digitalWrite(green,LOW);
      digitalWrite(blue1,LOW);                   
     }
    
  }
  delay(200);

}
