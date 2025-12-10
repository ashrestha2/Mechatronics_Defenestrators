#include <SPI.h>
#include <Pixy2.h>

Pixy2 pixy;

// PIXY SIGNATURE IDs
const int SIG_PINK = 1;
const int SIG_BLUE = 3;
const int SIG_PURPLE = 2;

// ========================================
// PIN DEFINITIONS
// ========================================

// Motor Pins
#define AIN1 13
#define AIN2 12
#define PWMA 11
#define BIN1 3
#define BIN2 4
#define PWMB 5

// IR Sensor Pins
#define SENSOR_PIN_LEFT 10
#define SENSOR_PIN_RIGHT 9

// Ultrasonic Sensor Pins
#define TRIG_PIN_FRONT 6
#define ECHO_PIN_FRONT 7
#define TRIG_PIN_LEFT 24
#define ECHO_PIN_LEFT 25
#define TRIG_PIN_RIGHT 22
#define ECHO_PIN_RIGHT 23

// Switch Pin
#define SWITCH_PIN 2

const int motor_pin = 8;
int intakeSpeed = 250;

// ========================================
// CONFIGURATION PARAMETERS
// ========================================

const int BASE_SPEED = 95;
const int GRAVEL_SPEED = 230;
const int HILL_SPEED = 35;
const int TURN_SPEED = 150;
const int WALL_FOLLOW_DISTANCE = 35;
const int FRONT_OBSTACLE_THRESHOLD = 38;
const int FRONT_OBSTACLE_THRESHOLD_GRAVEL = 25;
int TURN_DURATION = 125;

// ========================================
// STATE MACHINE DEFINITION
// ========================================

enum RobotState {
  STATE_WALL_FOLLOW,
  STATE_LINE_FOLLOW,
  STATE_GRAVEL,
  STATE_DOWNHILL
};

RobotState currentState = STATE_WALL_FOLLOW;

// ========================================
// GLOBAL VARIABLES
// ========================================

float distanceFront = 0;
float distanceLeft = 0;
float distanceRight = 0;
int switchState = LOW;

// Pixy timing variables
unsigned long lastPixyCheck = 0;
const unsigned long PIXY_CHECK_INTERVAL = 10;

// ========================================
// MOTOR CONTROL FUNCTIONS
// ========================================

void setLeftMotor(bool forward, int speed) {
  digitalWrite(BIN1, forward ? HIGH : LOW);
  digitalWrite(BIN2, forward ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

void setRightMotor(bool forward, int speed) {
  digitalWrite(AIN1, forward ? LOW : HIGH);
  digitalWrite(AIN2, forward ? HIGH : LOW);
  analogWrite(PWMA, speed);
}

void moveForward(int speed) {
  setLeftMotor(true, speed);
  setRightMotor(true, speed);
}

void turnLeft(int speed) {
  setLeftMotor(false, speed);
  setRightMotor(true, speed);
}

void turnRight(int speed) {
  setLeftMotor(true, speed);
  setRightMotor(false, speed);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ========================================
// SENSOR FUNCTIONS
// ========================================

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;
}

void updateUltrasonicReadings() {
  distanceFront = readUltrasonic(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
  distanceLeft = readUltrasonic(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  distanceRight = readUltrasonic(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
}

bool detectLine() {
  int leftSensor = digitalRead(SENSOR_PIN_LEFT);
  int rightSensor = digitalRead(SENSOR_PIN_RIGHT);
  return (leftSensor == LOW || rightSensor == LOW);
}

// ========================================
// PIXY CAM FUNCTIONS
// ========================================

bool detectSignature(int sig, int &x, int &y) {
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == sig) {
      x = pixy.ccc.blocks[i].m_x;
      y = pixy.ccc.blocks[i].m_y;
      return true;
    }
  }
  return false;
}

void checkColorSignatures() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastPixyCheck < PIXY_CHECK_INTERVAL) return;
  lastPixyCheck = currentMillis;

  pixy.ccc.getBlocks();

  int px, py, bx, by, prx, pry;
  bool seePink = detectSignature(SIG_PINK, px, py);
  bool seeBlue = detectSignature(SIG_BLUE, bx, by);
  bool seePurple = detectSignature(SIG_PURPLE, prx, pry);

  if (seePurple) {
    moveForward(90);
    seePink = detectSignature(SIG_PINK, px, py);
    seeBlue = detectSignature(SIG_BLUE, bx, by);
    seePurple = detectSignature(SIG_PURPLE, prx, pry);

    if (seePink && seePurple) {
      if (currentState != STATE_GRAVEL) {
        Serial.println("=== PINK + BLUE DETECTED - SWITCHING TO GRAVEL ===");
        currentState = STATE_GRAVEL;
      }
    } else if (seePurple && seeBlue && pry < 50) {
      if (currentState != STATE_DOWNHILL) {
        Serial.println("=== PURPLE DETECTED - SWITCHING TO DOWNHILL ===");
        currentState = STATE_DOWNHILL;
      }
    }
  }

  seePink = detectSignature(SIG_PINK, px, py);
  seeBlue = detectSignature(SIG_BLUE, bx, by);
  seePurple = detectSignature(SIG_PURPLE, prx, pry);

  if (seePink && seePurple) {
    if (currentState != STATE_GRAVEL) {
      Serial.println("=== PINK + BLUE DETECTED - SWITCHING TO GRAVEL ===");
      currentState = STATE_GRAVEL;
    }
  } else if (seePurple && seeBlue && pry < 50) {
    if (currentState != STATE_DOWNHILL) {
      Serial.println("=== PURPLE DETECTED - SWITCHING TO DOWNHILL ===");
      currentState = STATE_DOWNHILL;
    }
  }
}

// ========================================
// REUSABLE TURN FUNCTION
// ========================================

bool performTurnIfNeeded(int threshold) {
  updateUltrasonicReadings();

  if (distanceFront <= threshold) {
    if (distanceLeft > distanceRight) {
      turnLeft(TURN_SPEED + 20);
      delay(TURN_DURATION);
      checkColorSignatures();
      Serial.println("Front Obstacle - Turning Left");
    } else {
      turnRight(TURN_SPEED + 20);
      delay(TURN_DURATION);
      Serial.println("Front Obstacle - Turning Right");
    }
    return true;
  }
  return false;
}

// ========================================
// STATE: WALL FOLLOWING
// ========================================

void stateWallFollow() {
  checkColorSignatures();
  if (currentState != STATE_WALL_FOLLOW) return;

  if (detectLine()) {
    Serial.println("=== LINE DETECTED - SWITCHING TO LINE FOLLOW ===");
    currentState = STATE_LINE_FOLLOW;
    return;
  }

  if (performTurnIfNeeded(FRONT_OBSTACLE_THRESHOLD)) return;

  int error = distanceRight - distanceLeft;
  int turnAdjust = error * 6;

  int leftSpeed = BASE_SPEED + turnAdjust;
  int rightSpeed = BASE_SPEED - turnAdjust;

  setLeftMotor(true, constrain(leftSpeed, 65, 95));
  setRightMotor(true, constrain(rightSpeed, 65, 95));

  Serial.print("Wall Follow - L:");
  Serial.print(distanceLeft);
  Serial.print("cm R:");
  Serial.print(distanceRight);
  Serial.print("cm F:");
  Serial.print(distanceFront);
  Serial.println("cm");
}

// ========================================
// STATE: LINE FOLLOWING
// ========================================

void stateLineFollow() {
  checkColorSignatures();

  int leftSensor = digitalRead(SENSOR_PIN_LEFT);
  int rightSensor = digitalRead(SENSOR_PIN_RIGHT);

  if (leftSensor == HIGH && rightSensor == HIGH) {
    Serial.println("=== LINE LOST - SWITCHING TO WALL FOLLOW ===");
    currentState = STATE_WALL_FOLLOW;
    stopMotors();
    return;
  }

  if (leftSensor == HIGH && rightSensor == HIGH) {
    moveForward(BASE_SPEED);
  } else if (leftSensor == HIGH && rightSensor == LOW) {
    turnLeft(TURN_SPEED);
    delay(100);
  } else if (leftSensor == LOW && rightSensor == HIGH) {
    turnRight(TURN_SPEED);
    delay(100);
  }
}

// ========================================
// STATE: GRAVEL
// ========================================

void stateGravel(float gravelStartTime) {
  pixy.ccc.getBlocks();

  int px, py, bx, by;
  bool seePink = detectSignature(SIG_PINK, px, py);
  bool seeBlue = detectSignature(SIG_BLUE, bx, by);

  int leftSensor = digitalRead(SENSOR_PIN_LEFT);
  int rightSensor = digitalRead(SENSOR_PIN_RIGHT);

  if (performTurnIfNeeded(FRONT_OBSTACLE_THRESHOLD)) {
    if (distanceFront <= FRONT_OBSTACLE_THRESHOLD || (leftSensor == HIGH && rightSensor == LOW)) {
      if (distanceLeft > distanceRight) {
        turnLeft(TURN_SPEED + 20);
        delay(400);
        checkColorSignatures();
      } else {
        turnRight(TURN_SPEED + 20);
        delay(450);
      }

      setLeftMotor(true, GRAVEL_SPEED);
      setRightMotor(true, GRAVEL_SPEED);
      delay(2000);

      turnLeft(TURN_SPEED + 20);
      delay(650);
      checkColorSignatures();
      moveForward(135);

      currentState = STATE_WALL_FOLLOW;
    }
  }
}

// ========================================
// STATE: DOWNHILL
// ========================================

void stateDownhill() {
  pixy.ccc.getBlocks();

  int prx, pry;
  bool seePurple = detectSignature(SIG_PURPLE, prx, pry);

  updateUltrasonicReadings();

  if (distanceFront <= FRONT_OBSTACLE_THRESHOLD + 15) {
    Serial.println("=== TURN AHEAD - EXITING DOWNHILL MODE ===");
    performTurnIfNeeded(FRONT_OBSTACLE_THRESHOLD);
    currentState = STATE_WALL_FOLLOW;
    return;
  }

  int centerX = pixy.frameWidth / 2;
  int error = prx - centerX;
  int turnAdjust = error / 5;

  int leftSpeed = HILL_SPEED - turnAdjust;
  int rightSpeed = HILL_SPEED + turnAdjust;

  setLeftMotor(true, constrain(leftSpeed, 20, 50));
  setRightMotor(true, constrain(rightSpeed, 20, 50));
}

// ========================================
// SETUP
// ========================================

void setup() {
  Serial.begin(9600);
  pixy.init();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(SENSOR_PIN_LEFT, INPUT);
  pinMode(SENSOR_PIN_RIGHT, INPUT);

  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);

  pinMode(SWITCH_PIN, INPUT);
  pinMode(motor_pin, OUTPUT);

  stopMotors();
}

// ========================================
// MAIN LOOP
// ========================================

bool grav = false;
float gravelStartTime = 0;

void loop() {
  switchState = digitalRead(SWITCH_PIN);

  if (switchState == LOW) {
    stopMotors();
    digitalWrite(motor_pin, 0);
    return;
  }

  switch (currentState) {
    case STATE_WALL_FOLLOW:
      stateWallFollow();
      digitalWrite(motor_pin, intakeSpeed);
      grav = false;
      break;

    case STATE_LINE_FOLLOW:
      stateLineFollow();
      digitalWrite(motor_pin, intakeSpeed);
      grav = false;
      break;

    case STATE_GRAVEL:
      digitalWrite(motor_pin, 0);
      if (!grav) {
        gravelStartTime = millis();
        grav = true;
      }
      stateGravel(gravelStartTime);
      break;

    case STATE_DOWNHILL:
      stateDownhill();
      grav = false;
      break;
  }

  delay(10);
}
