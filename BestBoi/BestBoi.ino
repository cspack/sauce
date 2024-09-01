/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>
// Motor Setup ------------------
Servo lMotor;
Servo rMotor;

// Line Sensors ------------------
const int irSensorLPin = 4;
const int irSensorRPin = 6;
//Initialize values
int sensor1Value = 0;
int sensor2Value = 0;

//int potpin = A0;
//int potpin2 = A4;
//int val;
//int val2;

// Setup ------------------
void setup() {
  lMotor.attach(9);
  rMotor.attach(3);
  pinMode(irSensorLPin, INPUT);
  pinMode(irSensorRPin, INPUT);
  Serial.begin(9600);
}

// void loop() {
//   val = analogRead(potpin);
//   val2 = analogRead(potpin2);           // reads the value of the potentiometer (value between 0 and 1023)
//   val = map(val, 0, 1023, 1000, 2000);
//   val2 = map(val2, 0, 1023, 1000, 2000);     // scale it for use with the servo (value between 0 and 180)
//   myservo1.writeMicroseconds(val2);                  // sets the servo position according to the scaled value
//   myservo2.writeMicroseconds(val);
//   Serial.print(val);
//   Serial.print("\t val2 = ");
//   Serial.println(val2);                 // sets the servo position according to the scaled value
//   delay(15);                           // waits for the servo to get there
// }

void loop() {
  int RS = digitalRead(irSensorRPin);
  int LS = digitalRead(irSensorLPin);

  Serial.print("LS: ");
  Serial.print(LS);
  Serial.print(" RS: ");
  Serial.print(RS);

  if ((RS == 0) && (LS == 0)) { forward(); }
  if ((RS == 1) && (LS == 0)) { turnRight(); }
  if ((RS == 0) && (LS == 1)) { turnLeft(); }
  if ((RS == 1) && (LS == 1)) { Stop(); }

  // delay(15);
}


// Motor constants.
#define SERVO_STOP 1520
// Temporarily gate range to safer numbers:
#define SERVO_FULL_REVERSE 1480  // 1000
#define SERVO_REVERSE_RANGE -520
#define SERVO_FULL_FORWARD 1600  // 2000
#define SERVO_FORWARD_RANGE 480

void forward() {  //forward
  lMotor.writeMicroseconds(SERVO_FULL_FORWARD);
  rMotor.writeMicroseconds(SERVO_FULL_FORWARD);
}

void turnRight() {  //turnRight
  lMotor.writeMicroseconds(SERVO_FULL_FORWARD);
  rMotor.writeMicroseconds(SERVO_FULL_REVERSE);
}

void turnLeft() {  //turnLeft
  lMotor.writeMicroseconds(SERVO_FULL_REVERSE);
  rMotor.writeMicroseconds(SERVO_FULL_FORWARD);
}

void Stop() {
  // Loop in a circle?
  lMotor.writeMicroseconds(SERVO_STOP);
  rMotor.writeMicroseconds(SERVO_STOP);
}
