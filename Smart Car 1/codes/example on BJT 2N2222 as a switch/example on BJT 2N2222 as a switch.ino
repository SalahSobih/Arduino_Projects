int i=5 , j=6 , k=7 ; 


void setup() {
  // put your setup code here, to run once:
pinMode(i,OUTPUT);
pinMode(j,OUTPUT);
pinMode(k,OUTPUT);
}

void loop() {
digitalWrite(j,HIGH);
digitalWrite(i,HIGH);
digitalWrite(k,HIGH);
delay(3000);
digitalWrite(i,LOW);
digitalWrite(k,LOW);
delay(3000);
}
