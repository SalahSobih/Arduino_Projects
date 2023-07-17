#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050  car ; 

int16_t ax , ay , az ;
int16_t gx , gy , gz ;


void setup() {
  Serial.begin(9600);
  car.initialize();

}

void loop() {
  car.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gz = map(gz, -2000, 2000  , -500  , 500);
  int x =map(gz, -500  , 500 ,0 ,360);
  Serial.println(x);
delay(1000);
}
