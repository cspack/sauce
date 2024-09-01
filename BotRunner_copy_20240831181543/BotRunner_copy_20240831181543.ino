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
#define ROTATIONS_COLLECTED 100

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

// Motor Setup ------------------
Servo lMotor; 
Servo rMotor; 

// Debug printer:
// The last time that a Serial.print was made.
uint64_t lastTime = 0;

// Wheel data container.
struct Wheel {
  // Revolution collector:
  // The collection of timestamps of last revolutions.
  uint64_t ticks[ROTATIONS_COLLECTED];
  // The position to insert the next revolution.
  int index = 0;
  // The global revolutions for this motor since reset.
  int tickCount = 0;
  // Motor controller.
  Servo servo;

  void recordTick() {
    tickCount++;
    ticks[index] = millis();
    index = (++index) % ROTATIONS_COLLECTED;
  }

  int countTicks(uint64_t now, uint64_t window) {
    int count = 0;

    // Circuluar access to last node.
    int i = index - 1;
    if (i < 0) i += ROTATIONS_COLLECTED;

    uint64_t min_time = now - window;
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

void initWheel(Wheel wheel, int pin_servo, int pin_hall_effect) {
  // Setup Struct.
  wheel.index = 0;
  wheel.tickCount = 0;
  for (int i = 0; i < sizeof(wheel.ticks); i++) wheel.ticks[i] = -1;

  // Attach servo monitor.
  wheel.servo.attach(pin_servo);
}
Wheel left;
Wheel right;

void recieveHallEffectLeft() {
  left.recordTick();
}

void recieveHallEffectRight() {
  right.recordTick();
}

void setup() {
  // Set up wheel servos.
  initWheel(left, PIN_MOTOR_LEFT, PIN_HALL_EFFECT_LEFT);
  initWheel(right, PIN_MOTOR_RIGHT, PIN_HALL_EFFECT_RIGHT);

  // Set up IR sensors.
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_CENTER, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  // Set up the Hall Effect Sensors.
  pinMode(PIN_HALL_EFFECT_LEFT, INPUT);
  pinMode(PIN_HALL_EFFECT_RIGHT, INPUT);
  // Attach hall effect sensor.
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_LEFT), recieveHallEffectLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_RIGHT), recieveHallEffectRight, FALLING);

  Serial.begin(9600);
  Serial.println("Let's go!");
}

// AI told me this is how I print a uint64.
void printUint64(uint64_t value) {
  uint32_t high = value >> 32;
  uint32_t low = value & 0xFFFFFFFF;
  Serial.print(high, HEX);
  Serial.print(low, HEX);
}

void printDebugUpdate(uint64_t now) {
    Serial.print("[1s @");
    printUint64(now);
    Serial.print("] left ticks: ");
    //Serial.println(left.tickCount);
    Serial.print(left.countTicks(now, 1000));
    Serial.print(" right ticks: ");
    Serial.println(right.countTicks(now, 1000));
}

void executeDefaultLineRider() {
  // TODO: do something!
}

void executeStateMachine() {
  switch (currentPathState) {
    default:
    executeDefaultLineRider();
    break;
  }
}

void loop() {
  executeStateMachine();

  // Only give a status update every second.
  uint64_t now = millis();
  if ((now - lastTime) >= 1000) {
    printDebugUpdate(now);
    lastTime = now;
  }
}
