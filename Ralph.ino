#include <SPI.h>
#include <Pixy2.h>
Pixy2 pixy;


// PIXY SIGNATURE IDs
const int SIG_PINK   = 1;
const int SIG_BLUE   = 3;
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
#define SENSOR_PIN_LEFT  10
#define SENSOR_PIN_RIGHT 9

// Ultrasonic Sensor Pins
#define TRIG_PIN_FRONT 6
#define ECHO_PIN_FRONT 7
#define TRIG_PIN_LEFT  24
#define ECHO_PIN_LEFT  25
#define TRIG_PIN_RIGHT 22
#define ECHO_PIN_RIGHT 23

// Switch Pin
#define SWITCH_PIN 2

const int motor_pin = 8;
int intakeSpeed = 250;


// ========================================
// CONFIGURATION PARAMETERS
// ========================================

const int BASE_SPEED = 85;
const int GRAVEL_SPEED = 200;  // Increased speed for gravel
const int HILL_SPEED = 40;     // Reduced speed for downhill
const int TURN_SPEED = 120;
const int WALL_FOLLOW_DISTANCE = 35;
const int FRONT_OBSTACLE_THRESHOLD = 39;  // Default threshold
const int FRONT_OBSTACLE_THRESHOLD_GRAVEL = 25;  // Reduced threshold for gravel
int TURN_DURATION = 200;  // milliseconds - adjustable during runtime

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
const unsigned long PIXY_CHECK_INTERVAL = 10;  // Check Pixy every 100ms

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
  float distance = (duration * 0.0343) / 2;
  
  return distance;
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
  // Don't call getBlocks here - assume it's already been called
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
  // Only check Pixy at intervals to avoid spamming it
  unsigned long currentMillis = millis();
  if (currentMillis - lastPixyCheck < PIXY_CHECK_INTERVAL) {
    return;  // Skip this check
  }
  lastPixyCheck = currentMillis;
  
  // Get blocks once per check
  pixy.ccc.getBlocks();
  
  int px, py, bx, by, prx, pry;
  bool seePink = detectSignature(SIG_PINK, px, py);
  bool seeBlue = detectSignature(SIG_BLUE, bx, by);
  bool seePurple = detectSignature(SIG_PURPLE, prx, pry);
  
  // Gravel mode: Pink AND Blue together
  if (seePink && seePurple) {
    if (currentState != STATE_GRAVEL) {
      Serial.println("=== PINK + BLUE DETECTED - SWITCHING TO GRAVEL ===");
      currentState = STATE_GRAVEL;
    }
  }
  // Downhill mode: Purple only
  else if (seePurple && seeBlue) {
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
  
  // Check if there's a front obstacle
  if (distanceFront <= threshold) {
    // Decide which direction to turn based on side distances
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
    
    return true;  // Indicate that a turn was performed
  }
  
  return false;  // No turn needed
}

// ========================================
// STATE: WALL FOLLOWING
// ========================================

void stateWallFollow() {
  // Check for color signatures first
  checkColorSignatures();

  if (currentState != STATE_WALL_FOLLOW) {
    return;  // State changed, exit
  }
  
  // Check if we've reached a line
  if (detectLine()) {
    Serial.println("=== LINE DETECTED - SWITCHING TO LINE FOLLOW ===");
    currentState = STATE_LINE_FOLLOW;
    return;
  }
  
  // Check for front obstacle and turn if needed (35cm threshold)
  if (performTurnIfNeeded(FRONT_OBSTACLE_THRESHOLD)) {
    return;  // Turn was performed
  }
  
  // Normal wall following - proportional control
  int error = distanceRight - distanceLeft;
  int turnAdjust = error * 6;
  
  int leftSpeed = BASE_SPEED + turnAdjust;
  int rightSpeed = BASE_SPEED - turnAdjust;
  
  setLeftMotor(true, constrain(leftSpeed, 50, 90));
  setRightMotor(true, constrain(rightSpeed, 50, 90));
  
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
  int leftSensor = digitalRead(SENSOR_PIN_LEFT);
  int rightSensor = digitalRead(SENSOR_PIN_RIGHT);
  
  // Check if we've lost the line completely
  if (leftSensor == HIGH && rightSensor == HIGH) {
    Serial.println("=== LINE LOST - SWITCHING TO WALL FOLLOW ===");
    currentState = STATE_WALL_FOLLOW;
    stopMotors();
    return;
  }
  
  // Line following logic
  if (leftSensor == HIGH && rightSensor == HIGH) {
    moveForward(BASE_SPEED);
    Serial.println("Line: Forward");
  }
  else if (leftSensor == HIGH && rightSensor == LOW) {
    turnLeft(TURN_SPEED);
    Serial.println("Line: Turn Left");
  }
  else if (leftSensor == LOW && rightSensor == HIGH) {
    turnRight(TURN_SPEED);
    Serial.println("Line: Turn Right");
  }
}

// ========================================
// STATE: GRAVEL (PINK + BLUE SIGN)
// ========================================

void stateGravel(float gravelStartTime) {
  // Get fresh Pixy data
  pixy.ccc.getBlocks();
  
  int px, py, bx, by;
  bool seePink = detectSignature(SIG_PINK, px, py);
  bool seeBlue = detectSignature(SIG_BLUE, bx, by);
  
  // Update ultrasonic readings
  //updateUltrasonicReadings();
  
  
  
  // If we lose both pink and blue signs, return to wall follow
//  if (!seePink && !seeBlue) {
//    Serial.println("=== PINK/BLUE LOST - RETURNING TO WALL FOLLOW ===");
//    currentState = STATE_WALL_FOLLOW;
//    return;
//  }
  
//  // Calculate steering - use pink if available, otherwise blue
//  int targetX = seePink ? px : bx;
//  int centerX = pixy.frameWidth / 2;
//  int error = targetX - centerX;
//  int turnAdjust = error / 5;
//  
//  int leftSpeed = GRAVEL_SPEED - turnAdjust;
//  int rightSpeed = GRAVEL_SPEED + turnAdjust;

 if(performTurnIfNeeded(FRONT_OBSTACLE_THRESHOLD)){
  if (distanceFront <= FRONT_OBSTACLE_THRESHOLD) {
    // Decide which direction to turn based on side distances
    if (distanceLeft > distanceRight) {
      turnLeft(TURN_SPEED + 20);
      delay(450);
      checkColorSignatures();
      Serial.println("Front Obstacle - Turning Left");
    } else {
      turnRight(TURN_SPEED + 20);
      delay(450);
      Serial.println("Front Obstacle - Turning Right");
    }
      setLeftMotor(true,GRAVEL_SPEED);
      setRightMotor(true, GRAVEL_SPEED);
      delay(1000);
      currentState = STATE_WALL_FOLLOW;
      
     }
 
  Serial.print("GRAVEL MODE - Target at X:");
  //  Serial.print(targetX);
  //  Serial.print(" F:");
  //  Serial.print(distanceFront);
  //  Serial.print("cm L:");
  //  Serial.print(distanceLeft);
  //  Serial.print("cm R:");
  //  Serial.print(distanceRight);
  //  Serial.println("cm");
  }
}

// ========================================
// STATE: DOWNHILL (PURPLE SIGN)
// ========================================

void stateDownhill() {
  // Get fresh Pixy data
  pixy.ccc.getBlocks();
  
  int prx, pry;
  bool seePurple = detectSignature(SIG_PURPLE, prx, pry);
  
  // Update ultrasonic readings
  updateUltrasonicReadings();
  
  // Exit condition: Turn detected (front wall)
  if (distanceFront <= FRONT_OBSTACLE_THRESHOLD) {
    Serial.println("=== TURN AHEAD - EXITING DOWNHILL MODE ===");
    // Perform the turn before exiting
    performTurnIfNeeded(FRONT_OBSTACLE_THRESHOLD);
    currentState = STATE_WALL_FOLLOW;
    return;
  }
  
  // If we lose purple sign, return to wall follow
//  if (!seePurple) {
//    Serial.println("=== PURPLE LOST - RETURNING TO WALL FOLLOW ===");
//    currentState = STATE_WALL_FOLLOW;
//    return;
//  }
  
  // Calculate steering to keep purple sign centered
  int centerX = pixy.frameWidth / 2;
  int error = prx - centerX;
  int turnAdjust = error / 5;
  
  int leftSpeed = HILL_SPEED - turnAdjust;
  int rightSpeed = HILL_SPEED + turnAdjust;
  
  setLeftMotor(true, constrain(leftSpeed, 20, 60));
  setRightMotor(true, constrain(rightSpeed, 20, 60));
  
  Serial.print("DOWNHILL MODE - Purple at X:");
  Serial.print(prx);
  Serial.print(" F:");
  Serial.print(distanceFront);
  Serial.println("cm");
}

// ========================================
// SETUP
// ========================================

void setup() {
  Serial.begin(9600);
  Serial.println("===================================");
  Serial.println("Autonomous Bot - State Machine");
  Serial.println("===================================");
  
  // Initialize Pixy2
  pixy.init();
  Serial.println("Pixy2 initialized");
  
  Serial.println("Starting in WALL FOLLOW mode");
  
  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // IR sensor pins
  pinMode(SENSOR_PIN_LEFT, INPUT);
  pinMode(SENSOR_PIN_RIGHT, INPUT);
  
  // Ultrasonic pins
  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  
  // Switch pin
  pinMode(SWITCH_PIN, INPUT);
  pinMode(motor_pin, OUTPUT);
  
  stopMotors();
  Serial.println("Ready!");
}

// ========================================
// MAIN LOOP
// ========================================
  bool grav = false;
  float gravelStartTime = 0;
void loop() {
  // Check master switch
  switchState = digitalRead(SWITCH_PIN);
  
  if (switchState == LOW) {
    stopMotors();
    digitalWrite(motor_pin, 0);
    Serial.println("Power Off");
    return;
  }
  
  // Optional: Control intake motor
  // analogWrite(motor_pin, intakeSpeed);
  
  // Execute current state
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
      if(grav==false)
      {
        float gravelStartTime = millis();
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
