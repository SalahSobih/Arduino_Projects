int led = 13 ;
int sen = 4 ;


void setup() {
  pinMode(led,OUTPUT);
  pinMode(sen,INPUT);

}

void loop() {
  if (digitalRead(sen)==HIGH)
  {
    digitalWrite(led,HIGH);  
   }
   else
    digitalWrite(led,LOW); 
  
}
