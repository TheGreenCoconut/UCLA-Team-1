#include <Servo.h>
#define servo_pin 7
Servo swivel;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  swivel.attach(servo_pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  swivel.write(0);
}
