#include <SoftwareSerial.h>
SoftwareSerial car(10,11);

int motor1_left=4 , motor1_right=5 , motor2_left=6 , motor2_right=7 ;
int pwm1 = 24 , pwm2 = 26 , key = 12 ;
char control ;

unsigned int counter=0;
int dis_sen = 3 ;

int IRoutput;

void count()  
{
  counter++;  
} 

void setup() {
  Serial.begin(9600);
  car.begin(9600);
    
 pinMode(motor1_left,OUTPUT);
 pinMode(motor1_right,OUTPUT);
 pinMode(motor2_left,OUTPUT);
 pinMode(motor2_right,OUTPUT);
 pinMode(dis_sen,INPUT);

}

void loop() {

   if (car.available()==1)
 {   
  control=car.read();
  Serial.println( control); 
  if (control =='w' )
  while (1)
  {
    IRoutput = digitalRead ( 3 ) ;
    attachInterrupt ( digitalPinToInterrupt(3) , count , CHANGE ) ;
    Serial.println(counter) ;
       
    analogWrite(pwm1,100);
    analogWrite(pwm2,100);  
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    
    if(car.available()==1)
    break;
}
 }
 else if (control =='e' )
  {
    
    analogWrite(pwm1,180);
    analogWrite(pwm2,180);  
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
    
  }
  if (control =='r' )
  {
    
    analogWrite(pwm1,255);
    analogWrite(pwm2,255);  
    digitalWrite(motor1_left,HIGH);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,HIGH);
    digitalWrite(motor2_right,LOW);
       
  }     
else if (control == 's')
{
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);
  }
  
delay(25);
}
