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
#define LEFT_IN4 24
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

Servo swivel, leftClaw, rightClaw;
float distances[NUM_ANGLES] = {0};
float objDists[45] = {0};
float objPos[6] = {0};

bool foundHole = false;
bool pastWalls = false;

bool holeLeft = false;
bool holeRight = false;
bool holeStraight = false;

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }
  Serial.println("VL53L0X ready.");

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move the thing"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done\n");

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

  delay(3000);
}

void stopMotors()
{
  analogWrite(LEFT_ENA, 0);
  analogWrite(RIGHT_ENA, 0);
  analogWrite(LEFT_ENB, 0);
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
  fullPower();
}

void turnRight() {
  digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(LEFT_IN3, HIGH); digitalWrite(LEFT_IN4, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, HIGH); digitalWrite(RIGHT_IN4, LOW);
  fullPower();
}

void openClaw() {
  leftClaw.write(180);
  rightClaw.write(180);
  delay(200);
}

void closeClaw() {
  leftClaw.write(0);
  rightClaw.write(0);
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
    delay(350);
    distances[i] = getDistance();
    Serial.print("Angle ");
    Serial.print(degree);
    Serial.print(": ");
    Serial.println(distances[i]);

    if ((i == 0 || i == 18) && getDistance() > 45){
      pastWalls == true;
    }
  }
}

void findBestHole()
{
  holeLeft = false;
  holeRight = false;
  holeStraight = false;  

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
    if (center_angle >= 70 && center_angle <= 110) {
      Serial.println("STRAIGHT");
      foundHole = true;
      holeStraight = true;
    } else if (center_angle < 80) {
      Serial.println("LEFT");
      foundHole = true;
      holeLeft = true;
    } else {
      Serial.println("RIGHT");
      foundHole = true;
      holeRight = true;
    }
  } else {
    Serial.println("No valid hole found.");
  }
}

void avoidObstacle(){
    stopMotors();
    if (!holeStraight) {
      swivel.write(90);
      delay(200);
      while (getDistance() > 25){
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
           stopMotors();
            delay(220);
        if (getDistance() > 30) {
          goForward();
          delay(300);
          stopMotors();
        }
      } else {
        while (getDistance() < 30) {
          goForward();
        }
      }
}

void getObjects(char sector) {
  float objXPos[45] = {0};
  float objYPos[45] = {0};
  
  for (int i = 0; i < 45; i++) {
    objXPos[i] =  objDists[i] * cos((180-i) * DEG_TO_RAD);
    objYPos[i] = objDists[i] * sin((180-i) * DEG_TO_RAD);
  }
  for (int k = 0; k < 45; k++) {
    switch (sector) {
      case 'w':
        if ( (objXPos[k] > -73.0) && (objXPos[k] < -14.5) && (objYPos[k] < 53.0) && (objYPos[k] > 0.0 ) ) {
          Serial.println("object at ");
          Serial.print(objXPos[k]);
          objPos[0] = objXPos[k];
          Serial.print(", ");
          Serial.print(objYPos[k]);
          objPos[1] = objYPos[k];
          Serial.print(" ");
        } 
        break;
      case 's':
        if ( (objXPos[k] > -73.0) && (objXPos[k] < -14.5) && (objYPos[k] < 114.5) && (objYPos[k] > 53.0) ) {
          Serial.println("object at ");
          Serial.print(objXPos[k]);
          objPos[2] = objXPos[k];
          Serial.print(", ");
          Serial.print(objYPos[k]);
          objPos[3] = objYPos[k];
          Serial.print(" ");
        } 
        break;
      case 'n':
        if ( (objXPos[k] > 0.0) && (objXPos[k] < 57.0) && (objYPos[k] < 114.5) && (objYPos[k] > 53.0) ) {
          Serial.println("object at ");
          Serial.print(objXPos[k]);
          objPos[4] = objXPos[k];
          Serial.print(", ");
          Serial.print(objYPos[k]);
          objPos[5] = objYPos[k];
          Serial.print(" ");
        } 
        break;
      default:
        break;
    }
  }
}

void scan2() {
  for (int i = 180; i < 135; i--) {
    swivel.write(i);
    delay(80);
    objDists[180-i] = getDistance();
    
  }
}

void grabObject(char sector) {
  switch (sector) {
    case 's':
      openClaw();
      goRight();
      delay(700); //time it takes to get to the center
      stopMotors();
      goForward();
      delay(3000); //time it takes to get to the center of the squares
      stopMotors();
      goLeft();
      delay( objPos[3] - (114.5 / 2) ); //match ypos of object (NOTE: maybe multiply by constant)
      stopMotors();
      goForward();
      delay( objPos[2] - (114.5 / 2) ); //match xpos of object (NOTE: maybe multiply by constant)
      stopMotors();
      closeClaw();
      goRight();
      delay( objPos[3] - (114.5 / 2) ); //go to center again (NOTE: maybe multiply by constant)
      stopMotors();
      center();
      break;
    case 'n':
      openClaw();
      goRight();
      delay( 114.5 - objPos[5] ); //go to center again (NOTE: maybe multiply by constant)
      stopMotors();
      goForward();
      delay(3000 + (-objPos[4])); //time it takes to get to the front of the square plus the x position of the obj
      stopMotors();
      closeClaw();
      center();
      break;
    case 'w':
      openClaw();
      goRight();
      delay( 114.5 - objPos[5] ); //go to center again (NOTE: maybe multiply by constant)
      stopMotors();
      goForward();
      delay(4000 + (-objPos[4])); //time it takes to get to the front of the square plus the x position of the obj
      stopMotors();
      closeClaw();
      center();
      break;
    default:
      break;
  }
}

void center() {
  goLeft();
  delay(1000);
  stopMotors();
  goBackward();
  delay(2500);
  stopMotors();
  goLeft();
  delay(1000);
  stopMotors();
}

void loop()
{
  stopMotors();
  scan();
  if (!pastWalls) {
    findBestHole();
    if (foundHole == true){
      avoidObstacle();
    }
    delay(1500); // Pause for debug reading
  } else {
    // Stage 2 logic
    goRight();
    delay(1200);
    stopMotors();
    goLeft();
    delay(80);
    stopMotors();
    swivel.write(180);
    while (getDistance() < 20) {
      goForward();
      delay(10);
    }
    stopMotors();
    scan2();
    
    getObjects('s');
    goForward();
    delay(1000);
    stopMotors();
    turnRight();
    delay(500);
    stopMotors();

    center();
    grabObject('s');
  }
}
