#include <Servo.h>

// Digital pins.
#define PIN_IR_LEFT 4
#define PIN_IR_CENTER 5
#define PIN_IR_RIGHT 6
#define PIN_HALL_EFFECT_LEFT 7
#define PIN_HALL_EFFECT_RIGHT 8
#define PIN_LASER_POWER 10
#define PIN_MOTOR_LEFT 9
#define PIN_MOTOR_RIGHT 3
// Analog pins.
#define PIN_LASER_READER 0

// Configs.
#define ROTATIONS_COLLECTED 10

#define RUN_MOTORS true
// Motor constants.
#define SERVO_STOP 1520
// Temporarily gate range to safer numbers:
#define SERVO_FULL_REVERSE 1200  // 1000
#define SERVO_FULL_FORWARD 1800  // 2000

#define DEBUG_WRITE_SAMPLE_RATE 1000
#define CONTROLLER_SAMPLE_RATE 1

// Define State Machine.
enum PathState {
  START,
  CHECKPOINT_ONE,
  CHECKPOINT_TWO,
  BROOM_STICK_BEGIN,
  BROOM_STICK_ABYSS,
  JUMP_EXIT_LANE,
  DO_A_FLIP,
  FIND_THE_LINE,
  RED_LINE_RIDER,
  INVERTED_CUP_STRAIGHT,
  INVERTED_CUP_RIGHT,
  FINISH
};
PathState currentPathState = START;

// The last time that a Serial.print was made.
uint32_t lastPrintTime = 0;
uint32_t lastExecTime = 0;

struct InfraredWrapper {
  int pin = -1;
  int isBlack = false;
  // The time this bit last flipped.
  uint32_t flippedAt = 0;
  // Assume that the interrupt handles both directions of type CHANGE.
  void handleChange() {
    isBlack = digitalRead(pin);
    flippedAt = millis();
  }
};

// Wheel data container.
struct Wheel {
  int pinServo = -1;
  int pinHallEffect = -1;
  // Revolution collector:
  // The collection of timestamps of last revolutions.
  uint32_t myTicks[ROTATIONS_COLLECTED];
  // The position to insert the next revolution.
  int index = 0;
  // The global revolutions for this motor since reset.
  int tickCount = 0;
  // Motor controller.
  Servo* myServo;

  void recordTick() {
    tickCount++;
    myTicks[index] = millis() % 0xFFFFFFFF;
    index = (++index) % ROTATIONS_COLLECTED;
  }

  #define EPSILON 0.00001
  bool areSame(double a, double b) {
    return fabs(a - b) < EPSILON;
    ;
  }


  // sets the servo speed: range -1 to 1.
  void setSpeed(double rate) {
    if (!RUN_MOTORS) {
      return;
    }
    // Fast base cases:
    if (areSame(rate, 0.0)) {
      myServo->writeMicroseconds(SERVO_STOP);
    }
    if (areSame(rate, 1.0)) {
      myServo->writeMicroseconds(SERVO_FULL_FORWARD);
    }
    if (areSame(rate, -1.0)) {
      myServo->writeMicroseconds(SERVO_FULL_REVERSE);
    }
    if (rate > 0.0) {
      myServo->writeMicroseconds(SERVO_STOP + (rate * abs(SERVO_FULL_FORWARD - SERVO_STOP)));
    } else {
      myServo->writeMicroseconds(SERVO_STOP - (rate * abs(SERVO_STOP - SERVO_FULL_REVERSE)));
    }
  }

  int countTicks(uint32_t now, uint32_t window) {
    int count = 0;

    // Circuluar access to last node.
    int i = index - 1;
    if (i < 0) i += ROTATIONS_COLLECTED;

    uint32_t min_time = now - window;
    if (min_time < 0)
      return 0;

    while (true) {
      if (myTicks[i] >= min_time) {
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
};

void initWheel(Wheel& wheel, int pin_servo, int pin_hall_effect) {
  Serial.print("Initializing Wheel/servo on pin ");
  Serial.println(pin_servo);
  // wheel.pinServo = pin_servo;
  // wheel.pinHallEffect = pin_hall_effect;

  // Setup Struct.
  wheel.index = 0;
  wheel.tickCount = 0;
  for (int i = 0; i < sizeof(wheel.myTicks); i++) wheel.myTicks[i] = -1;
}
// Instances of wheel.
Wheel leftWheel;
Wheel rightWheel;

// Interrupt implementations.
void recieveHallEffectLeft() {
  leftWheel.recordTick();
}
void recieveHallEffectRight() {
  rightWheel.recordTick();
}

void initIrSensor(InfraredWrapper& irSensor, int irPin) {
  irSensor.pin = irPin;
  pinMode(irPin, INPUT);
  irSensor.isBlack = digitalRead(irPin);
  irSensor.flippedAt = millis();
}
InfraredWrapper leftIr;
InfraredWrapper centerIr;
InfraredWrapper rightIr;

void recieveIrLeft() {
  leftIr.handleChange();
}
void recieveIrCenter() {
  centerIr.handleChange();
}
void recieveIrRight() {
  rightIr.handleChange();
}

Servo leftServo;
Servo rightServo;

void setup() {
  Serial.begin(9600);

  leftServo.attach(PIN_MOTOR_LEFT);
  rightServo.attach(PIN_MOTOR_RIGHT);
  leftWheel.myServo = &leftServo;
  rightWheel.myServo = &rightServo;
  if (!leftServo.attached()) {
      Serial.println("Left servo failed attach.");
      delay(10000);
  }
  if (!rightServo.attached()) {
      Serial.println("Right servo failed attach.");
      delay(10000);
  }

  // Set up wheel servos.
  initWheel(leftWheel, PIN_MOTOR_LEFT, PIN_HALL_EFFECT_LEFT);
  initWheel(rightWheel, PIN_MOTOR_RIGHT, PIN_HALL_EFFECT_RIGHT);

  // Set up IR sensors.
  initIrSensor(leftIr, PIN_IR_LEFT);
  initIrSensor(centerIr, PIN_IR_CENTER);
  initIrSensor(rightIr, PIN_IR_RIGHT);
  /*
  if (digitalPinToInterrupt(PIN_IR_LEFT) < 0) {
    Serial.println("NO INTERRUPT ON PIN PIN_IR_LEFT");
  } else {
    attachInterrupt(digitalPinToInterrupt(PIN_IR_LEFT), recieveIrLeft, CHANGE);
  }
  if (digitalPinToInterrupt(PIN_IR_CENTER) < 0) {
    Serial.println("NO INTERRUPT ON PIN PIN_IR_CENTER");
  } else {
    attachInterrupt(digitalPinToInterrupt(PIN_IR_CENTER), recieveIrCenter, CHANGE);
  }
  if (digitalPinToInterrupt(PIN_IR_RIGHT) < 0) {
    Serial.println("NO INTERRUPT ON PIN PIN_IR_RIGHT");
  } else {
    attachInterrupt(digitalPinToInterrupt(PIN_IR_RIGHT), recieveIrRight, CHANGE);
  }*/

  /*
  // Set up the Hall Effect Sensors.
  pinMode(PIN_HALL_EFFECT_LEFT, INPUT);
  pinMode(PIN_HALL_EFFECT_RIGHT, INPUT);*/
  // Attach hall effect sensor.
  // attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_LEFT), recieveHallEffectLeft, FALLING);
  // attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_RIGHT), recieveHallEffectRight, FALLING);

  Serial.println("Let's go!");
}

void printDebugUpdate(uint32_t now) {
  Serial.print("[1s @");
  Serial.print(now);
  Serial.print("] sensor L:");
  Serial.print(leftIr.isBlack ? "Y" : "N");
  Serial.print(",C:");
  Serial.print(centerIr.isBlack ? "Y" : "N");
  Serial.print(",R:");
  Serial.print(rightIr.isBlack ? "Y" : "N");
  Serial.println();

/*
Serial.print(" LEFT: ");
  Serial.print(digitalRead(PIN_IR_LEFT));
  Serial.print(" RIGHT: ");
  Serial.println(digitalRead(PIN_IR_RIGHT));*/

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
  leftWheel.setSpeed(-0.2);
  rightWheel.setSpeed(-0.2);
};
void moveLeft() {
  leftWheel.setSpeed(-1.0);
  rightWheel.setSpeed(1.0);
};
void moveRight() {
  leftWheel.setSpeed(1.0);
  rightWheel.setSpeed(-1.0);
};
void moveForward() {
  leftWheel.setSpeed(1.0);
  rightWheel.setSpeed(1.0);
};

void executeDefaultLineRider(uint32_t timeSinceStart) {
  // Silent debug.
  if (!RUN_MOTORS) {
    return;
  }

  if (rightIr.isBlack && leftIr.isBlack) {
      moveForward();
  }
  if (leftIr.isBlack) {
    moveRight();
    return;
  }
  if (rightIr.isBlack) {
    moveLeft();
    return;
  }
  moveForward();
}

void executeStateMachine(uint32_t timeSinceStart) {
  switch (currentPathState) {
    case START:
      executeDefaultLineRider(timeSinceStart);
      break;
  }
}

void refreshSensors() {
  leftIr.handleChange();
  centerIr.handleChange();
  rightIr.handleChange();
}

void loop() {
  uint32_t now = millis() & 0xFFFFFFFF;

  if ((now - lastExecTime) >= CONTROLLER_SAMPLE_RATE) {
    refreshSensors();

    int timeSinceStart = now - lastExecTime;
    executeStateMachine(timeSinceStart);
    lastExecTime = now;
  }

  // Only give a status update every second.
  if ((now - lastPrintTime) >= DEBUG_WRITE_SAMPLE_RATE) {
    printDebugUpdate(now);
    lastPrintTime = now;
  }
}
