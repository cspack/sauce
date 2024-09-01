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
// The last time the user went slow because scared of the dark.
uint32_t lastInch = 0;
// How long we slow turning after last inching
#define WAS_JUST_GOING_SLOW 2000

bool disableMotors() {
  return false;
}

uint32_t millis_32() {
  return millis() % 0xFFFFFFFF;
}
// Assume that the interrupt handles both directions of type CHANGE.
static void detectInfraredChange(InfraredWrapper& ir) {
  int oldValue = ir.isBlack;
  ir.isBlack = digitalRead(ir.pin);
  if (oldValue != ir.isBlack) {
    ir.flippedAt = millis_32();
  }
}

#define EPSILON 0.0001
static bool areSame(float a, float b) {
  return fabs(a - b) < EPSILON;
}

int wheelLogTick(Wheel& wheel) {
  wheel.tickCount++;
  wheel.ticks[wheel.index] = millis_32();
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
  if (disableMotors()) {
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
  irSensor.flippedAt = millis_32();
  irSensor.reverse = reverse;
}

void setup() {
  Serial.begin(9600);

  // Set up wheel servos.
  leftServo.attach(PIN_MOTOR_LEFT);
  rightServo.attach(PIN_MOTOR_RIGHT);
  wheelInit(leftWheel, PIN_MOTOR_LEFT, PIN_HALL_EFFECT_LEFT, &leftServo);
  wheelInit(rightWheel, PIN_MOTOR_RIGHT, PIN_HALL_EFFECT_RIGHT, &rightServo);
  // Breathe.
  delay(1000);

  wheelSetSpeed(leftWheel, 0.0);
  wheelSetSpeed(rightWheel, 0.0);
  delay(1000);
  // There is no spoon.

  // Set up IR sensors.
  initIrSensor(leftIr, PIN_IR_LEFT, /*reverse=*/false);
  initIrSensor(centerIr, PIN_IR_CENTER, /*reverse=*/false);
  // GG.
  initIrSensor(rightIr, PIN_IR_RIGHT, /*reverse=*/true);

  // Set up the Hall Effect Sensors.
  pinMode(PIN_HALL_EFFECT_LEFT, INPUT);
  pinMode(PIN_HALL_EFFECT_RIGHT, INPUT);

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

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

  // Distance monitoring. For another day.
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
  if (millis_32() - lastInch < WAS_JUST_GOING_SLOW) {
    wheelSetSpeed(leftWheel, -0.3);
    wheelSetSpeed(rightWheel, 0.3);
    return;
  }
  // Serial.println("GO LEFT!");
  wheelSetSpeed(leftWheel, -0.5);
  wheelSetSpeed(rightWheel, 0.5);
};
void moveRight() {
  if (millis_32() - lastInch < WAS_JUST_GOING_SLOW) {
    wheelSetSpeed(leftWheel, 0.3);
    wheelSetSpeed(rightWheel, -0.3);
    return;
  }
  // Serial.println("GO RIGHT!");
  wheelSetSpeed(leftWheel, 0.5);
  wheelSetSpeed(rightWheel, -0.5);
};

void moveTightLeft() {
  Serial.println("GO TIGHT LEFT!");
  wheelSetSpeed(leftWheel, -1.0);
  wheelSetSpeed(rightWheel, 0.2);
};
void moveTightRight() {
  Serial.println("GO TIGHT RIGHT!");
  wheelSetSpeed(leftWheel, 0.2);
  wheelSetSpeed(rightWheel, -1.0);
};
void inchForward() {
  lastInch = millis_32();
  wheelSetSpeed(leftWheel, 0.3);
  wheelSetSpeed(rightWheel, 0.3);
}
void moveForward() {
  // Serial.println("GO FORWARD!");
  wheelSetSpeed(leftWheel, 0.5);
  wheelSetSpeed(rightWheel, 0.5);
};

bool isAtACrossRoad() {
  return (rightIr.isBlack && leftIr.isBlack && centerIr.isBlack);
}

bool isInHeaven() {
  return (!rightIr.isBlack && !leftIr.isBlack && !centerIr.isBlack);
}

void writeRgb(bool red, bool green, bool blue) {
  digitalWrite(PIN_RED, red ? HIGH : LOW);
  digitalWrite(PIN_GREEN, green ? HIGH : LOW);
  digitalWrite(PIN_BLUE, blue ? HIGH : LOW);
}

bool isOnTheLine() {
  return (!rightIr.isBlack && !leftIr.isBlack && centerIr.isBlack);
}
uint32_t enteredCheckpoint = 0;

#define MINIMUM_STATE_WAIT 6000
uint32_t lastStateChange = 0;
void advanceState(PathState newState, int waitTime = MINIMUM_STATE_WAIT) {

  int32_t now = millis_32();
  if (now - waitTime > lastStateChange) {
    Serial.print("STATE CHANGE: ");
    Serial.println(newState);

    // RGB Status!
    switch(newState) {
      case CHECKPOINT_ONE:
        writeRgb(true, false, false);
        break;
      case CHECKPOINT_TWO:
        writeRgb(false, true, false);
        break;
      case GET_ON_THE_LINE:
        writeRgb(false, false, true);
        break;
      case EXIT_TO_THE_RAMP:
        // PINK
        writeRgb(true, false, true);
        break;
      case DO_A_FLIP:
        // CYAN
        writeRgb(false, true, true);
        break;
      case FIND_THE_LINE:
        // ORANGE
        writeRgb(true, true, false);
        break;
      case RED_LINE_RIDER:
        writeRgb(true, false, false);
        break;
      case INVERTED_CUP_STRAIGHT:
        writeRgb(false, true, false);
        break;
      case INVERTED_CUP_RIGHT:
        writeRgb(false, false, true);
        break;
    }

    lastStateChange = millis_32();
    currentPathState = newState;
    moveStop();
    delay(1000);
  }
}

#define CHECKPOINT_ONE_CROSS_WAIT_TIME 6
#define CHECKPOINT_TWO_CROSS_WAIT_TIME 6
void checkpointDetector() {
  if (currentPathState == START) {
    if (isAtACrossRoad()) {
      if (enteredCheckpoint == 0) {
        enteredCheckpoint = millis_32();
      }
    } else if (enteredCheckpoint == 0) {
      // Do nothing
    } else {
      // Evaluate wether hit checkpoint when LEFT the crossroad.
      if (millis_32() - enteredCheckpoint >= CHECKPOINT_ONE_CROSS_WAIT_TIME) {
        advanceState(CHECKPOINT_ONE);
      }
      // Reset.
      enteredCheckpoint = 0;
    }
  }

  if (currentPathState == CHECKPOINT_ONE) {
    if (isAtACrossRoad()) {
      if (enteredCheckpoint == 0) {
        enteredCheckpoint = millis_32();
      }
    } else if (enteredCheckpoint == 0) {
      // Do nothing
    } else {
      // Evaluate wether hit checkpoint when LEFT the crossroad.
      if (millis_32() - enteredCheckpoint >= CHECKPOINT_TWO_CROSS_WAIT_TIME) {
        advanceState(CHECKPOINT_TWO);
      }
      // Reset.
      enteredCheckpoint = 0;
    }
  }
  if (currentPathState == CHECKPOINT_TWO) {
    if (isInHeaven()) {
      advanceState(BROOM_STICK_ABYSS);
    }
  }
  if (currentPathState == BROOM_STICK_ABYSS) {
    if (!isInHeaven()) {
      advanceState(GET_ON_THE_LINE);
    }
  }
  if (currentPathState == GET_ON_THE_LINE) {
    if (isOnTheLine()) {
      // Allow fast advance because this is a quick process.
      advanceState(EXIT_TO_THE_RAMP, 1000);
    }
  }
}

#define STOP_LEFT_FRAMES_NEEDED 3
int stopLeftInstances = 0;
int stopLeftFrames = 0;
void checkAdvance() {
  stopLeftFrames++;
  if (stopLeftFrames >= STOP_LEFT_FRAMES_NEEDED) {
    stopLeftInstances++;
    stopLeftFrames = 0;
  }
  if (stopLeftInstances >= 2) {
    advanceState(CHECKPOINT_TWO);
  }
}

void executeDefaultLineRider() {
  uint32_t now = millis_32();
  if (rightIr.isBlack && leftIr.isBlack) {
    //if (currentPathState == START) {
      moveStop();
      moveLeft();
      checkAdvance();
      return;
    /*} else {
      inchForward();
      return;
    }*/
  } else {
    if (currentPathState == START) {
      stopLeftInstances--;
    }
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

void executePreferRightLineRider() {
  if (rightIr.isBlack && leftIr.isBlack) {
    //if (currentPathState == START) {
      moveStop();
      moveRight();
      checkAdvance();
      return;/*
    } else {
      inchForward();
      return;
    }*/
  } else {
    if (currentPathState == START) {
      stopLeftInstances--;
    }
  }
  if (rightIr.isBlack) {
    moveRight();
    return;
  }
  if (leftIr.isBlack) {
    moveLeft();
    return;
  }
  moveForward();
}

void executeRideTheRightLineRider() {
  if ((!leftIr.isBlack || rightIr.isBlack) && (millis_32() % 8 == 0)) {
    moveRight();
  }
  if (centerIr.isBlack && !rightIr.isBlack) {
    moveLeft();
    return;
  }
  if (rightIr.isBlack) {
    moveRight();
    return;
  }
  if (centerIr.isBlack) {
    moveLeft();
    return;
  }
  moveForward();
}
void executeVeerLeftLineRider() {
  if (rightIr.isBlack && leftIr.isBlack) {
    moveTightLeft();
    return;
  }
  if (leftIr.isBlack) {
    moveTightLeft();
    return;
  }
  if (rightIr.isBlack) {
    moveRight();
    return;
  }
  moveForward();
}
void executeVeerRightLineRider() {
  if (rightIr.isBlack && leftIr.isBlack) {
    moveTightRight();
    return;
  }
  if (rightIr.isBlack) {
    moveTightRight();
    return;
  }
  if (leftIr.isBlack) {
    moveLeft();
    return;
  }
  moveForward();
}



void executeStateMachine() {
  if (lastStateChange == 0) {
    lastStateChange = millis_32();
  }
  checkpointDetector();
  switch (currentPathState) {
    case START:
      executeDefaultLineRider();
      if (millis_32() - STAGE_ONE_TIME > lastStateChange) {
        advanceState(CHECKPOINT_ONE);
      }
      break;
    case CHECKPOINT_ONE:
      executeDefaultLineRider();
      break;
    case CHECKPOINT_TWO:
      executePreferRightLineRider();
      break;
    case BROOM_STICK_ABYSS:
      executeVeerRightLineRider();
      break;
    case GET_ON_THE_LINE:
      executePreferRightLineRider();
      break;
    case EXIT_TO_THE_RAMP:
      executeVeerLeftLineRider();
      if (millis_32() - 2000 > lastStateChange) {
        advanceState(DO_A_FLIP);
      }
      break;
    case DO_A_FLIP:
      executeDefaultLineRider();
      break;
    default:
      executeDefaultLineRider();
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
