int i=9;
void setup() {
  pinMode(i,OUTPUT);
}

void loop() {
  digitalWrite(i,HIGH);
  delay(500);
  digitalWrite(i,LOW);
  delay(1500);
}
