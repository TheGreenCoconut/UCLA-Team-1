#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_VL53L0X.h>

MPU6050 mpu(Wire);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
unsigned long timer = 0;

// --- Pin definitions ---
#define SERVO_PIN 2
#define LEFT_CLAW_PIN 6
#define RIGHT_CLAW_PIN 7

#define LEFT_ENA 34
#define LEFT_IN1 32
#define LEFT_IN2 28
#define LEFT_ENB 22
#define LEFT_IN3 26
#define LEFT_IN4 25
#define RIGHT_ENA 8
#define RIGHT_IN1 9
#define RIGHT_IN2 10
#define RIGHT_ENB 13
#define RIGHT_IN3 11
#define RIGHT_IN4 12

// --- Scan definitions ---
#define NUM_ANGLES 19
#define ANGLE_STEP 10
#define WINDOW_SIZE 3 // 3*10=30deg minimum hole width

// --- Drive Constants ---
const float cmps = 7.85;
// const float dps = 202.25;

Servo swivel, leftClaw, rightClaw;
float distances[NUM_ANGLES] = {0};



bool foundHole = false;
bool pastWalls = false;

bool holeLeft = false;
bool holeRight = false;
bool holeOffsetStraight = false;
bool holeStraight = false;

int obstacleCount =0;
int numObstacles = 4;

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move the thing"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done\n");

  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }
  Serial.println("VL53L0X ready.");

  swivel.attach(SERVO_PIN);
  leftClaw.attach(LEFT_CLAW_PIN);
  rightClaw.attach(RIGHT_CLAW_PIN);

  pinMode(LEFT_ENA, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_ENB, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);

  pinMode(RIGHT_ENA, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  openClaw();

  delay(3000);
}

void stopMotors()
{
  analogWrite(LEFT_ENA, 0);
  analogWrite(LEFT_ENB, 0);
  analogWrite(RIGHT_ENA, 0);
  delay(80);
  analogWrite(RIGHT_ENB, 0);
}

void fullPower()
{
  analogWrite(LEFT_ENA, 255);
  analogWrite(LEFT_ENB, 255);
  analogWrite(RIGHT_ENA, 255);
  analogWrite(RIGHT_ENB, 255);
}

void halfPower()
{
  analogWrite(LEFT_ENA, 0);
  analogWrite(LEFT_ENB, 0);
  analogWrite(RIGHT_ENA, 120);
  analogWrite(RIGHT_ENB, 120);
}

void goForward()
{
  digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH); digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, HIGH);
  fullPower();
}

void goBackward()
{
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, LOW); digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
  fullPower();
}

void goRight()
{
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, HIGH); digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, HIGH);
  fullPower();
}

void goLeft()
{
  digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, LOW); digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
  fullPower();
}

void turnLeft() {
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN3, LOW); digitalWrite(LEFT_IN4, HIGH);
  digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW); digitalWrite(RIGHT_IN4, HIGH);
    halfPower();
}

void turnRight() {
  digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH); digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
    halfPower();
}

void openClaw() {
  leftClaw.write(27);
  rightClaw.write(215);
  delay(200);
}

void closeClaw() {
  leftClaw.write(95);
  rightClaw.write(140);
  delay(200);
}

float getDistance()
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return ((measure.RangeMilliMeter) / 10.0);
  } else {
    return 819;
  }
}

void scan()
{
  // Updates the values in distances
  for (int i = 0; i < NUM_ANGLES; i++) {
    int degree = i * ANGLE_STEP;
    swivel.write(degree);
    delay(200);
    distances[i] = getDistance();
    Serial.print("Angle ");
    Serial.print(degree);
    Serial.print(": ");
    Serial.println(distances[i]);

    // if ((i == 0 || i == 19) && getDistance() > 100){
    //   pastWalls = true;
    // }
  }
}

void findBestHole()
{
  holeLeft = false;
  holeRight = false;
  holeStraight = false;  
  holeOffsetStraight = false;

  // Ignore 0, 10, 170, and 180 (indices 0, 1, 17, 18)
  // Use sliding window (default: 3) from indices 2 to 16-WINDOW_SIZE+1
  int best_start = -1;
  float best_avg = -1;
  int min_idx = 2;
  int max_idx = NUM_ANGLES - WINDOW_SIZE - 2; // e.g. 16 for window=3

  for (int i = min_idx; i <= NUM_ANGLES - WINDOW_SIZE - 2 + 1; i++) {
    float sum = 0;
    for (int w = 0; w < WINDOW_SIZE; w++) {
      sum += distances[i + w];
    }
    float avg = sum / WINDOW_SIZE;
    if (avg > best_avg) {
      best_avg = avg;
      best_start = i;
    }
  }

  if (best_start != -1) {
    int center_idx = best_start + WINDOW_SIZE / 2;
    int center_angle = center_idx * ANGLE_STEP;
    Serial.print("Best hole at ");
    Serial.print(center_angle);
    Serial.print(" deg (avg dist ");
    Serial.print(best_avg, 1);
    Serial.print(" cm): ");
    if (center_angle > 110 ) {
      Serial.println("RIGHT");
      foundHole = true;
      holeRight = true;
    } else if (center_angle < 70) {
      Serial.println("LEFT");
      foundHole = true;
      holeLeft = true;
    } else if (center_angle >= 80 && center_angle<=100){
      Serial.println("STRAIGHT");
      foundHole = true;
      holeStraight = true;
    } else {
        if (center_angle == 70){
          Serial.println("OFFSET STRAIGHT LEFT");
          foundHole = true;
          holeOffsetStraight = true;
          holeLeft = true;
        }
        if (center_angle == 110){
          Serial.println("OFFSET STRAIGHT RIGHT");
          foundHole = true;
          holeOffsetStraight = true;
          holeRight = true;
        }
    }
  } else {
    Serial.println("No valid hole found.");
  }
}

void correctYaw(float targetAngle) {
  float currentAngle = mpu.getAngleZ();
  while (currentAngle > targetAngle + 1){
    mpu.update();
    currentAngle = mpu.getAngleZ();
    Serial.println(currentAngle);
    turnRight();
  }
  while (currentAngle < targetAngle - 1){
    mpu.update();
    currentAngle = mpu.getAngleZ();
    Serial.println(currentAngle);
    turnLeft();
  }
  stopMotors();
}

void avoidObstacle(){
    stopMotors();
      swivel.write(90);
      delay(200);
    if (!holeStraight) {
      if (!holeOffsetStraight) {
        while (getDistance() > 32){
          goForward();
        }
        stopMotors();
        delay(500);
        while (getDistance() < 40) {
          if (holeLeft){
            goLeft();
          } else {
            goRight();
          }
        }
        delay(100);
        stopMotors();
        delay(220);
        if (getDistance() > 30) {
          goForward();
          delay(150);
          stopMotors();
        }
      } else {
        if (holeLeft){
          goLeft();
          delay(275);
        } else {
          goRight();
          delay(275);
        }
        stopMotors();
        delay(220);
        if (getDistance() > 30) {
          goForward();
          delay(160);
          stopMotors();
        }
      }
    } else {
      goForward();
      delay(350);
      stopMotors();
      }
      
}

void loop()
{
  stopMotors();
  correctYaw(0);
  delay(1000);
  scan();
  if (!pastWalls) {
    
    findBestHole();
    if (foundHole == true){
      avoidObstacle();
      obstacleCount+=1;
      if (obstacleCount >= numObstacles){
        pastWalls=true;
      }
    }
    delay(1500); // Pause for debug reading
  } else {
    if (obstacleCount == numObstacles){
      goForward();
      delay(1500);
      stopMotors();
      goRight();
      delay(5000);
      stopMotors();
      obstacleCount += 1;
    }
    // Stage 2 logic (not modified)
  }
}
