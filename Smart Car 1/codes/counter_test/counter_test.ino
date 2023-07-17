int x =26 ;
int counter ;

void docount()  
{
  counter++;  
} 
void setup() {
  Serial.begin(9600);
  pinMode(x,INPUT);

}

void loop() {
  counter =0 ;
  while (counter< 10)
  {
int IRoutput = digitalRead ( x ) ;
attachInterrupt ( digitalPinToInterrupt(x) , docount , CHANGE ) ;
Serial.println(counter) ;
  }
       
}
