#include <Servo.h>

// Digital pins.
#define PIN_IR_LEFT 4
#define PIN_IR_CENTER 5
#define PIN_IR_RIGHT 6
#define PIN_HALL_EFFECT_LEFT 7
#define PIN_HALL_EFFECT_RIGHT 8
#define PIN_LASER_POWER 10
#define PIN_MOTOR_LEFT 3
#define PIN_MOTOR_RIGHT 9
// Analog pins.
#define PIN_LASER_READER 0

// Configs.
#define ROTATIONS_COLLECTED 10

// Motor constants.
#define SERVO_STOP 1520
// Temporarily gate range to safer numbers:
#define SERVO_FULL_REVERSE 1420  // 1000
#define SERVO_FULL_FORWARD 1620  // 2000

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

// Debug printer:
// The last time that a Serial.print was made.
uint32_t lastTime = 0;

struct InfraredWrapper {
  int pin = -1;
  int isBlack = false;
  // The time this bit last flipped.
  uint32_t flippedAt = 0;
  // Assume that the interrupt handles both directions of type CHANGE.
  void handleChange() {
    isBlack = !isBlack;
    flippedAt = millis();
  }
};

// Wheel data container.
struct Wheel {
  int pinServo = -1;
  int pinHallEffect = -1;
  // Revolution collector:
  // The collection of timestamps of last revolutions.
  uint32_t ticks[ROTATIONS_COLLECTED];
  // The position to insert the next revolution.
  int index = 0;
  // The global revolutions for this motor since reset.
  int tickCount = 0;
  // Motor controller.
  Servo servo;

  void recordTick() {
    tickCount++;
    ticks[index] = millis() % 0xFFFFFFFF;
    index = (++index) % ROTATIONS_COLLECTED;
  }

  // sets the servo speed: range -1 to 1.
  void setSpeed(double rate) {
    // Fast base cases:
    if (rate == 0.0) {
      servo.writeMicroseconds(SERVO_STOP);
    }
    if (rate == 1.0) {
      servo.writeMicroseconds(SERVO_FULL_FORWARD);
    }
    if (rate == -1.0) {
      servo.writeMicroseconds(SERVO_FULL_REVERSE);
    }
    if (rate > 0) {
      servo.writeMicroseconds(SERVO_STOP + rate * SERVO_FULL_FORWARD);
    } else {
      servo.writeMicroseconds(SERVO_STOP + rate * SERVO_FULL_REVERSE);
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
      if (ticks[i] >= min_time) {
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
  wheel.pinServo = pin_servo;
  wheel.pinHallEffect = pin_hall_effect;
  // Setup Struct.
  wheel.index = 0;
  wheel.tickCount = 0;
  for (int i = 0; i < sizeof(wheel.ticks); i++) wheel.ticks[i] = -1;

  // Attach servo monitor.
  wheel.servo.attach(pin_servo);
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

void setup() {
  // Set up wheel servos.
  initWheel(leftWheel, PIN_MOTOR_LEFT, PIN_HALL_EFFECT_LEFT);
  initWheel(rightWheel, PIN_MOTOR_RIGHT, PIN_HALL_EFFECT_RIGHT);

  // Set up IR sensors.
  initIrSensor(leftIr, PIN_IR_LEFT);
  initIrSensor(centerIr, PIN_IR_CENTER);
  initIrSensor(rightIr, PIN_IR_RIGHT);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_LEFT), recieveIrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_CENTER), recieveIrCenter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_RIGHT), recieveIrRight, CHANGE);

  // Set up the Hall Effect Sensors.
  pinMode(PIN_HALL_EFFECT_LEFT, INPUT);
  pinMode(PIN_HALL_EFFECT_RIGHT, INPUT);
  // Attach hall effect sensor.
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_LEFT), recieveHallEffectLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_RIGHT), recieveHallEffectRight, FALLING);

  Serial.begin(9600);
  Serial.println("Let's go!");
}

void printDebugUpdate(uint32_t now) {
  Serial.print("[1s @");
  Serial.print(now);
  Serial.print("] left wheel ∑:");
  Serial.print(leftWheel.tickCount);
  Serial.print(" Δ:");
  Serial.print(leftWheel.countTicks(now, 1000));
  Serial.print("right wheel ∑:");
  Serial.print(rightWheel.tickCount);
  Serial.print(" Δ:");
  Serial.print(rightWheel.countTicks(now, 1000));
}

void moveStop() {
  leftWheel.setSpeed(0);
  rightWheel.setSpeed(0);
};
void moveLeft() {
  leftWheel.setSpeed(-1);
  rightWheel.setSpeed(1);
};
void moveRight() {
  leftWheel.setSpeed(1);
  rightWheel.setSpeed(-1);
};
void moveForward() {
  leftWheel.setSpeed(1);
  rightWheel.setSpeed(1);
};

void executeDefaultLineRider() {
  if (rightIr.isBlack && leftIr.isBlack) {
    moveStop();
    return;
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

void executeStateMachine() {
  switch (currentPathState) {
    case START:
      executeDefaultLineRider();
      break;
  }
}

void loop() {
  executeStateMachine();

  // Only give a status update every second.
  uint32_t now = millis() % 0xFFFFFFFF;
  if ((now - lastTime) >= 1000) {
    printDebugUpdate(now);
    lastTime = now;
  }
}
