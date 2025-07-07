//NOTE: see if the floats are exact or not
// If an object is grabbed, use our yaw to angle the back of the robot to the back wall (-180). Drive back until our acceleration through the gyro is minimal while sending it voltag
// Then strafe to the back corner closest to the hard coded drop spot (same logic as reversing to the wall). Pathfind from that corner to the dropsppot using hard coded distance

#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

#define trig_pin 6
#define echo_pin 5

#define left_servo_pin 6
#define right_servo_pin 7

#define servo_pin 2

float left_cm, right_cm;
float distances[180] = {0};
int16_t x, y, yaw;

Servo swivel;
Servo leftClaw;
Servo rightClaw;

#define left_enA 21
#define left_in1 20
#define left_in2 19

#define left_enB 16
#define left_in3 18
#define left_in4 17

#define right_enA 8
#define right_in1 9
#define right_in2 10

#define right_enB 13
#define right_in3 11
#define right_in4 12

void setup()
{
  Serial.begin(9600);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move the thing"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done\n");

  swivel.attach(servo_pin);
  leftClaw.attach(left_servo_pin);
  rightClaw.attach(right_servo_pin);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  pinMode(left_enA, OUTPUT);
  pinMode(left_in1, OUTPUT);
  pinMode(left_in2, OUTPUT);
  pinMode(left_enB, OUTPUT);
  pinMode(left_in3, OUTPUT);
  pinMode(left_in4, OUTPUT);

  pinMode(right_enA, OUTPUT);
  pinMode(right_in1, OUTPUT);
  pinMode(right_in2, OUTPUT);
  pinMode(right_enB, OUTPUT);
  pinMode(right_in3, OUTPUT);
  pinMode(right_in4, OUTPUT);
}

void stopMotors()
{
  analogWrite(left_enA, 0);
  analogWrite(right_enA, 0);
  analogWrite(left_enB, 0);
  delay(80);
  analogWrite(right_enB, 0);
}

void fullPower()
{
  analogWrite(left_enA, 255);
  analogWrite(left_enB, 255);
  analogWrite(right_enA, 255);
  analogWrite(right_enB, 255);
}

void goForward()
{
  digitalWrite(left_in1, HIGH);
  digitalWrite(left_in2, LOW);
  digitalWrite(left_in3, HIGH);
  digitalWrite(left_in4, LOW);

  digitalWrite(right_in1, HIGH);
  digitalWrite(right_in2, LOW);
  digitalWrite(right_in3, LOW);
  digitalWrite(right_in4, HIGH);

  fullPower();
}

void goBackward()
{
  digitalWrite(left_in1, LOW);
  digitalWrite(left_in2, HIGH);
  digitalWrite(left_in3, LOW);
  digitalWrite(left_in4, HIGH);

  digitalWrite(right_in1, LOW);
  digitalWrite(right_in2, HIGH);
  digitalWrite(right_in3, HIGH);
  digitalWrite(right_in4, LOW);

  fullPower();
}

void goLeft()
{
  digitalWrite(left_in1, LOW);
  digitalWrite(left_in2, HIGH);
  digitalWrite(left_in3, HIGH);
  digitalWrite(left_in4, LOW);

  digitalWrite(right_in1, LOW);
  digitalWrite(right_in2, HIGH);
  digitalWrite(right_in3, LOW);
  digitalWrite(right_in4, HIGH);
  
  fullPower();
}

void goRight()
{
  digitalWrite(left_in1, HIGH);
  digitalWrite(left_in2, LOW);
  digitalWrite(left_in3, HIGH);
  digitalWrite(left_in4, LOW);

  digitalWrite(right_in1, LOW);
  digitalWrite(right_in2, HIGH);
  digitalWrite(right_in3, HIGH);
  digitalWrite(right_in4, LOW);

  fullPower();
}

void turnLeft(){
  digitalWrite(left_in1, LOW);
  digitalWrite(left_in2, HIGH);
  digitalWrite(left_in3, LOW);
  digitalWrite(left_in4, HIGH);

  digitalWrite(right_in1, HIGH);
  digitalWrite(right_in2, LOW);
  digitalWrite(right_in3, LOW);
  digitalWrite(right_in4, HIGH);

  fullPower();
}

void turnRight(){
  digitalWrite(left_in1, HIGH);
  digitalWrite(left_in2, LOW);
  digitalWrite(left_in3, HIGH);
  digitalWrite(left_in4, LOW);

  digitalWrite(right_in1, LOW);
  digitalWrite(right_in2, HIGH);
  digitalWrite(right_in3, HIGH);
  digitalWrite(right_in4, LOW);

  fullPower();
}

void openClaw()
{
  leftClaw.write(180);
  rightClaw.write(180);
  delay(200);
}

void closeClaw()
{
  leftClaw.write(0);
  rightClaw.write(0);
  delay(200);
}

float getDistance()
{
  // Returns distance in centimeters. 18 points are plotted in this way at once
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  return (pulseIn(echo_pin, HIGH)/2)/29.1;
}

void scan()
{
  // Updates the values in distances
  for (int degree = 0; degree <= 180; degree += 1) {
    swivel.write(degree);
    delay(50);

    distances[degree] = getDistance();
    Serial.println(distances[degree]);
    
  }
}

void calcHoleWidth() {
  
  for (int position = 0; position < 180; position++) {
    if((distances[position]-distances[position-1])>15){
      Serial.print("Start hole at");
      Serial.println(position);
      Serial.println("");
    }
  }
}
  /*
  for (int i = 1; i < 18; i++) {
    interDists[i] = sqrt(pow(xPositions[i]-xPositions[i-1], 2) + pow(yPositions[i]-yPositions[i-1],2));
  }
  
  for (int i = 1; i < 18; i++) {
    if ((abs(xPositions[i] - xPositions[i-1]) > 30) || (abs(yPositions[i] - yPositions[i-1]) > 20)) {
        Serial.println(xPositions[i]);
        Serial.print(" to ");
        Serial.print(xPositions[i-1]);
        Serial.print("cm x wise");
    }
  }
}
/*
float* getIrregularities()
{
  float average;
  float irregularities[18] = {0};
  for (int i = 0; i < 18; i ++)
  {
    average += distances[i];
  }
  average /= 18;
  for (int i = 0; i < 18; i++)
  {
    irregularities[i] = distances[i] - average;
  }
  return irregularities;
}
*/

void scoreObjectL1(){
  closeClaw();
  goLeft();
  delay(100);
  if (abs(mpu.getAccX()) < 0.05) {
    stopMotors();
    goBackward();
  }
  delay(100);
  if (abs(mpu.getAccY()) < 0.05) {
    stopMotors();
    goForward();
    delay(1250);
    goRight();
    delay(1000);
    openClaw();
  }
}

void scoreObjectL2(){
  closeClaw();
  goLeft();
  delay(100);
  if (abs(mpu.getAccX()) < 0.05) {
    stopMotors();
    goBackward();
  }
  delay(100);
  if (abs(mpu.getAccY()) < 0.05) {
    stopMotors();
    goForward();
    delay(1250);
    goRight();
    delay(1000);
    openClaw();
  }
}

void loop()
{
  /*
  mpu.update();
  if ((millis() - timer) > 10) {
    yaw = mpu.getAngleZ();
  */
  scan();
  calcHoleWidth();
  
}