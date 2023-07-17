#include <SoftwareSerial.h>
SoftwareSerial car(10,11);

int motor1_left=30 , motor1_right=32 , motor2_left=34 , motor2_right=36 ;
int pwm1 = 4 , pwm2 = 5 , key = 12 ;
char control ;

char InputDigit ;
int counter ;
int dis_sen = 3 ;

float InputDistance ;
String InputString ;
float InputFloat ;
float Input ;

void docount()  
{
  counter++;  
} 
float GetInput ()
{
  InputString = "" ;
  while ( 1 )
  {
    if ( car.available() == 1 )
    {
      InputDigit = car.read() ;
      if ( InputDigit != 'E' )
      {
        InputString += String(InputDigit) ;
      }
      else
      {
        break ;
      }
    }
  }
  InputFloat = atof ( InputString.c_str() ) ;
  return InputFloat ;
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
  if (control =='D' ) 
  {
     InputDistance = GetInput() ; 
     float c = (InputDistance/1.45);
     if (InputDistance<0)
     c=-c ;
     counter =0 ;
     while (counter < c)
     {
     if (InputDistance>0)
     {
     analogWrite(pwm1,255);
     analogWrite(pwm2,255);
     digitalWrite(motor1_left,HIGH);
     digitalWrite(motor1_right,LOW);
     digitalWrite(motor2_left,HIGH);
     digitalWrite(motor2_right,LOW);
     }
     else 
     {
     analogWrite(pwm1,255);
     analogWrite(pwm2,255);
     digitalWrite(motor1_left,LOW);
     digitalWrite(motor1_right,HIGH);
     digitalWrite(motor2_left,LOW);
     digitalWrite(motor2_right,HIGH);
     }
     
    attachInterrupt ( digitalPinToInterrupt(3) , docount , CHANGE ) ;
    Serial.println(counter) ;
  } 
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW); 
}
  else if(control=='s')
  {   
    digitalWrite(motor1_left,LOW);
    digitalWrite(motor1_right,LOW);
    digitalWrite(motor2_left,LOW);
    digitalWrite(motor2_right,LOW);     
  }    
}  
delay(25);
}
