int control=A1;
int dir1=40 , dir2=42 ; 
int motor=10;
int sp;
int sw=38;
void setup()
{
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(sw,INPUT);
}
void loop()
{
  if (digitalRead(sw)==HIGH)
   {
    digitalWrite(dir2,LOW);
    delay (1000);
    digitalWrite(dir1,HIGH); 
   }
  else
   {
     digitalWrite(dir1,LOW);
    delay (1000);   
    digitalWrite(dir2,HIGH);
   }
   sp=analogRead(control)/4.0;
   analogWrite(motor,sp);
}
