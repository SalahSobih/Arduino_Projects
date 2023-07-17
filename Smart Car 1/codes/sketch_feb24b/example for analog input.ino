int led=1 ; 
int control=A7 ;
int help ;
void setup() {
 pinMode(led,OUTPUT);
}

void loop() {
help = analogRead(control);
digitalWrite(led,HIGH);
delay(help);
digitalWrite(led,LOW);
delay(help);
}
