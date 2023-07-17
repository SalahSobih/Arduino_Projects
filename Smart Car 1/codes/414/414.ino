#include <SoftwareSerial.h>
SoftwareSerial leds(10,11);
int led1=30 , led2=32 , led3=34 , led4=36 , key = 12 ;
char test ;

void setup() {
Serial.begin(9600);
leds.begin(9600);
pinMode(led1,OUTPUT);
pinMode(led2,OUTPUT);
pinMode(led3,OUTPUT);
pinMode(led4,OUTPUT);
 
  }

void loop() {
  digitalWrite(key,HIGH);
  if (leds.available()==1)
  {
    test = leds.read();
    Serial.write(test);
    if (test == 'a') 
    {
  digitalWrite(led1,HIGH);
  digitalWrite(led2,LOW);
  digitalWrite(led3,HIGH);
  digitalWrite(led4,LOW);
    }
    else if (test == 'b')
    {
digitalWrite(led1,LOW);
  digitalWrite(led2,HIGH);
  digitalWrite(led3,LOW);
  digitalWrite(led4,HIGH);
    }
    else if (test == 'c')
    {
digitalWrite(led1,HIGH);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);
  digitalWrite(led4,LOW);
    }
    else if (test == 'd')
    {
digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(led3,HIGH);
  digitalWrite(led4,LOW);
    }
     else if (test == 's')
    {
digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);
  digitalWrite(led4,LOW);
    }

  }
  delay (100);
}
