#include <Servo.h>

// TCS230/TCS3200 pins wiring to Arduino
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

// Servo pin
#define SERVO_PIN 11

// Switch pin
#define SWITCH_PIN 2

// Frequency readings
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// RGB mapped values
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

int switchState = LOW;

// Servo object
Servo myServo;

void setup() {
  // Set color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Attach servo
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Start at center position

  // Switch pin
  pinMode(SWITCH_PIN, INPUT);

  // Begin serial communication
  Serial.begin(9600);
}

void loop() {
  // Check master switch
  switchState = digitalRead(SWITCH_PIN);

  // Read RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  
  redColor = map(redFrequency, 200, 1600, 255, 0);
  redColor = constrain(redColor, 0, 255);
  Serial.print(redFrequency);
  Serial.print(" R = ");
  Serial.print(redColor);
  //Serial.print(redColor);
  delay(100);

  // Read GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  greenColor = map(greenFrequency, 300, 1200, 255, 0);
  greenColor = constrain(greenColor, 0, 255);
  Serial.print(" G = ");
  Serial.print(greenColor);
  delay(100);

  // Read BLUE
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  blueColor = map(blueFrequency, 200, 1000, 255, 0);
  blueColor = constrain(blueColor, 0, 255);
  Serial.print(" B = ");
  Serial.print(blueColor);
  delay(100);

  if (switchState == LOW) {
    // Determine the dominant color
    if (redColor > greenColor && redColor > blueColor && abs(redColor - greenColor) > 20) {
      Serial.println(" - RED detected!");
      myServo.write(90);  // Reset to center
      delay(500);
      myServo.write(20);   // Turn CCW
      delay(1500);        // Hold
      myServo.write(90);  // Return to center
    }
    else if (greenColor > redColor && greenColor > blueColor && abs(greenColor - redColor) > 20) {
      Serial.println(" - GREEN detected!");
      myServo.write(90);  // Reset to center
      delay(500);
      myServo.write(160); // Turn CW
      delay(1500);        // Hold
      myServo.write(90);  // Return to center
    }
    else {
      Serial.println(" - No valid color (BLUE or MIX). Doing nothing.");
      // Do nothing, keep at center
      myServo.write(90);   // Turn CCW

    }
  }
  else {
    // Determine the dominant color
    if (redColor > greenColor && redColor > blueColor && abs(redColor - greenColor) > 20) {
      Serial.println(" - RED detected!");
      myServo.write(90);  // Reset to center
      delay(500);
      myServo.write(160); // Turn CW
      delay(1500);        // Hold
      myServo.write(90);  // Return to center
    }
    else if (greenColor > redColor && greenColor > blueColor && abs(greenColor - redColor) > 20) {
      Serial.println(" - GREEN detected!");
      myServo.write(90);  // Reset to center
      delay(500);
      myServo.write(20);   // Turn CCW
      delay(1500);        // Hold
      myServo.write(90);  // Return to center
    }
    else {
      Serial.println(" - No valid color (BLUE or MIX). Doing nothing.");
      // Do nothing, keep at center
      myServo.write(90);   // Turn CCW

    }
  }
  

  delay(2000); // Wait before next sensing cycle
}
