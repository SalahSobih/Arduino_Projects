int i=8 , j=9 , k=10 , l=11 ;


void setup() {
 pinMode(i,OUTPUT);
 pinMode(j,OUTPUT);
 pinMode(k,OUTPUT);
 pinMode(l,OUTPUT);
}

void loop() {
 digitalWrite(i,HIGH);
 delay (500);
 digitalWrite(j,HIGH);
 delay (500);
 digitalWrite(k,HIGH);
 delay (500);
 digitalWrite(l,HIGH);
 delay (500);
 digitalWrite(i,LOW);
 delay (500);
 digitalWrite(j,LOW);
 delay (500);
 digitalWrite(k,LOW);
 delay (500);
 digitalWrite(l,LOW);
 delay (500);

}
