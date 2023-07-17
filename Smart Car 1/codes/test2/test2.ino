#include <SoftwareSerial.h> 
SoftwareSerial project (12,13);
 int key=11;
 int  red=5 , blue=6 ;
char control; 
 
 void setup() {
 Serial.begin(9600);
 Serial.println("entre data");
 project.begin(9600);
 pinMode(red,OUTPUT);
 pinMode(blue,OUTPUT);

}

void loop() {
  digitalWrite(key,HIGH);
 if(project.available()==1)
 {
  control=project.read();
  if(control=='f')
  {
    
    digitalWrite(blue,HIGH); 
    digitalWrite(red,LOW);
    project.println("blue on");   
   }
 else if(control=='b')
  {
   
    digitalWrite(red,HIGH); 
    digitalWrite(blue,LOW); 
    project.println("red on");  
   }  
  
 }
delay(200);
}
