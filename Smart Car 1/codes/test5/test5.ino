#include <SoftwareSerial.h> 
SoftwareSerial project (11,12);
const int key=11;
const int dir1_mor1=30 , dir2_mor1=32 , dir1_mor2=34 , dir2_mor2=36 ;
int color=3 , red1=4 , blue=5 , yellow=6 , red2=7 ; 
const int pwm1=9 , pwm2=8 ;
char control;

void setup() {
  Serial.begin(9600);
 Serial.println("entre data");
 project.begin(9600);
 pinMode(dir1_mor1,OUTPUT);
 pinMode(dir2_mor1,OUTPUT);
 pinMode(pwm1,OUTPUT);
 pinMode(dir1_mor2,OUTPUT);
 pinMode(dir2_mor2,OUTPUT);
 pinMode(color,OUTPUT);
 pinMode(red1,OUTPUT);
 pinMode(blue,OUTPUT);
 pinMode(yellow,OUTPUT);
 pinMode(red2,OUTPUT);
 
}

void loop() {
   digitalWrite(key,HIGH);
 if(project.available()==1)
 {
   digitalWrite(color,HIGH);
   analogWrite(pwm1,100);
   analogWrite(pwm2,100);
  control=project.read();
   if(control=='f')
  {
 digitalWrite(dir1_mor1,LOW);
 digitalWrite(dir2_mor1,HIGH);
 digitalWrite(dir1_mor2,HIGH);
 digitalWrite(dir2_mor2,LOW);
 digitalWrite(blue,HIGH);
 digitalWrite(red1,LOW);
 digitalWrite(red2,LOW);
 digitalWrite(yellow,LOW);  
 }
 else if(control=='b')
  {
 digitalWrite(dir1_mor1,HIGH);
 digitalWrite(dir2_mor1,LOW);
 digitalWrite(dir1_mor2,LOW);
 digitalWrite(dir2_mor2,HIGH); 
 digitalWrite(blue,LOW);
 digitalWrite(red1,LOW);
 digitalWrite(red2,LOW);
 digitalWrite(yellow,HIGH); 
 }
 else if(control=='r')
  {
 digitalWrite(dir1_mor1,HIGH);
 digitalWrite(dir2_mor1,LOW);
 digitalWrite(dir1_mor2,HIGH);
 digitalWrite(dir2_mor2,LOW);
 digitalWrite(blue,LOW);
 digitalWrite(red1,HIGH);
 digitalWrite(red2,LOW);
 digitalWrite(yellow,LOW);
 }
else if(control=='l')
  {
 digitalWrite(dir1_mor1,LOW);
 digitalWrite(dir2_mor1,HIGH);
 digitalWrite(dir1_mor2,LOW);
 digitalWrite(dir2_mor2,HIGH);
 digitalWrite(blue,LOW);
 digitalWrite(red1,LOW);
 digitalWrite(red2,HIGH);
 digitalWrite(yellow,LOW);  
 }

 else if(control=='s')
  {
 digitalWrite(dir1_mor1,LOW);
 digitalWrite(dir2_mor1,LOW);
 digitalWrite(dir1_mor2,LOW);
 digitalWrite(dir2_mor2,LOW);
 digitalWrite(blue,LOW);
 digitalWrite(red1,LOW);
 digitalWrite(red2,LOW);
 digitalWrite(yellow,LOW);  
 }
 
 }
 delay(200); 
}
