#include <Servo.h>

#include "BusyBoi.h"

PathState currentPathState = START;
// Instances of sensor wrappers:
Wheel leftWheel;
Wheel rightWheel;
Servo leftServo;
Servo rightServo;
InfraredWrapper leftIr;
InfraredWrapper centerIr;
InfraredWrapper rightIr;
// The last time that a Serial.print was made:
uint32_t lastPrintTime = 0;
uint32_t lastExecTime = 0;

// Assume that the interrupt handles both directions of type CHANGE.
static void detectInfraredChange(InfraredWrapper& ir) {
  int oldValue = ir.isBlack;
  ir.isBlack = digitalRead(ir.pin);
  if (oldValue != ir.isBlack) {
    ir.flippedAt = millis();
  }
}

#define EPSILON 0.0001
static bool areSame(float a, float b) {
  return fabs(a - b) < EPSILON;
}

int wheelLogTick(Wheel& wheel) {
  wheel.tickCount++;
  wheel.ticks[wheel.index] = millis() % 0xFFFFFFFF;
  wheel.index = (++wheel.index) % ROTATIONS_COLLECTED;
  return 0;
}

int convertToServoValue(float rate) {
  // We're going in reverse now!!
  rate = rate * -1;
  if (rate >= 0.0) {
    return SERVO_CENTER + (rate * abs(SERVO_FORWARD_MULTIPLIER * SERVO_FULL_FORWARD));
  } 
  return SERVO_CENTER + (rate * abs(SERVO_REVERSE_MULTIPLIER * SERVO_FULL_REVERSE));
}

// sets the servo speed: range -1 to 1.
void wheelSetSpeed(Wheel& wheel, float rate) {
  if (!RUN_MOTORS) {
    return;
  }
  wheel.servo->writeMicroseconds(convertToServoValue(rate));
}

// Counts the number of revolutions since a specified time window (in millis).
int countWheelTicks(Wheel& wheel, uint32_t now, uint32_t window) {
  int count = 0;

  // Circuluar access to last node.
  int i = wheel.index - 1;
  if (i < 0) i += ROTATIONS_COLLECTED;

  uint32_t min_time = now - window;
  if (min_time < 0)
    return 0;

  while (true) {
    if (wheel.ticks[i] >= min_time) {
      count++;
      if (count > ROTATIONS_COLLECTED) {
        // FUCK!
        break;
      }

      i--;
      if (i < 0) i += ROTATIONS_COLLECTED;
    } else {
      // Expected.
      break;
    }
  }
  return count;
}

void wheelInit(Wheel& wheel, int pin_servo, int pin_hall_effect, Servo* servo) {
  wheel.pinServo = pin_servo;
  Serial.print("Initializing Wheel/servo on pin ");
  Serial.println(wheel.pinServo);
  wheel.servo = servo;

  // Setup struct fields.
  wheel.index = 0;
  wheel.tickCount = 0;
  for (int i = 0; i < ROTATIONS_COLLECTED; i++) wheel.ticks[i] = -1;
}


void initIrSensor(InfraredWrapper& irSensor, int irPin, bool reverse) {
  irSensor.pin = irPin;
  pinMode(irPin, INPUT);
  irSensor.isBlack = digitalRead(irPin);
  irSensor.flippedAt = millis();
  irSensor.reverse = reverse;
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LASER_POWER, OUTPUT);
  digitalWrite(PIN_LASER_POWER, HIGH);

  // Set up wheel servos.
  leftServo.attach(PIN_MOTOR_LEFT);
  rightServo.attach(PIN_MOTOR_RIGHT);
  wheelInit(leftWheel, PIN_MOTOR_LEFT, PIN_HALL_EFFECT_LEFT, &leftServo);
  wheelInit(rightWheel, PIN_MOTOR_RIGHT, PIN_HALL_EFFECT_RIGHT, &rightServo);

  // Set up IR sensors.
  initIrSensor(leftIr, PIN_IR_LEFT, /*reverse=*/ false);
  initIrSensor(centerIr, PIN_IR_CENTER, /*reverse=*/ false);
  initIrSensor(rightIr, PIN_IR_RIGHT, /*reverse=*/ true);

  // Set up the Hall Effect Sensors.
  pinMode(PIN_HALL_EFFECT_LEFT, INPUT);
  pinMode(PIN_HALL_EFFECT_RIGHT, INPUT);
  Serial.println("Let's a go!");
}

void printDebugUpdate() {
  Serial.print("sensor L:");
  Serial.print(leftIr.isBlack ? "Y" : "N");
  Serial.print(",C:");
  Serial.print(centerIr.isBlack ? "Y" : "N");
  Serial.print(",R:");
  Serial.print(rightIr.isBlack ? "Y" : "N");
  Serial.println();

  // Distance monitoring:
  /*
  Serial.print("] left wheel ∑:");
  Serial.print(leftWheel.tickCount);
  Serial.print(" Δ:");
  Serial.print(leftWheel.countTicks(now, 1000));
  Serial.print("right wheel ∑:");
  Serial.print(rightWheel.tickCount);
  Serial.print(" Δ:");
  Serial.println(rightWheel.countTicks(now, 1000));
  */
}

void moveStop() {
  Serial.println("GO STOP!");
  wheelSetSpeed(leftWheel, 0.0);
  wheelSetSpeed(rightWheel, 0.0);
};
void moveLeft() {
  Serial.println("GO LEFT!");
  wheelSetSpeed(leftWheel, -1.0);
  wheelSetSpeed(rightWheel, 1.0);
};
void moveRight() {
  Serial.println("GO RIGHT!");
  wheelSetSpeed(leftWheel, 1.0);
  wheelSetSpeed(rightWheel, -1.0);
};
void moveForward() {
  Serial.println("GO FORWARD!");
  wheelSetSpeed(leftWheel, 1.0);
  wheelSetSpeed(rightWheel, 1.0);
};

void executeDefaultLineRider() {
  if (rightIr.isBlack && leftIr.isBlack) {
    moveStop();
  }
  if (leftIr.isBlack) {
    moveLeft();
    return;
  }
  if (rightIr.isBlack) {
    moveRight();
    return;
  }
  moveForward();
}

void executeStateMachine() {
  switch (currentPathState) {
    case START:
      executeDefaultLineRider();
      break;
  }
}

void refreshSensors() {
  detectInfraredChange(leftIr);
  detectInfraredChange(centerIr);
  detectInfraredChange(rightIr);
}

void loop() {
  refreshSensors();
  
  uint32_t now = millis() & 0xFFFFFFFF;
  if ((now - lastExecTime) >= CONTROLLER_SAMPLE_RATE) {
    int timeSinceStart = now - lastExecTime;
    executeStateMachine();
    lastExecTime = now;
  }

  // Only give a status update every second.
  if ((now - lastPrintTime) >= DEBUG_WRITE_SAMPLE_RATE) {
    printDebugUpdate();
    lastPrintTime = now;
  }
}
