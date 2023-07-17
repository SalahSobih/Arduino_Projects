int i=2 , j=3 , k=4 ;
void setup() {
 pinMode(i,OUTPUT);
 pinMode(j,OUTPUT);
 pinMode(k,OUTPUT);
}

void loop() {
  digitalWrite(i,HIGH);
  delay(2000);

  digitalWrite(i,LOW);
  digitalWrite(j,HIGH);
  delay(2000);

  digitalWrite(j,LOW);
  digitalWrite(k,HIGH);
  delay(2000);

  digitalWrite(k,LOW);
}
