#include <Servo.h>

// Digital pins.
#define PIN_IR_LEFT 5
#define PIN_IR_CENTER 4
#define PIN_IR_RIGHT 6
#define PIN_HALL_EFFECT_LEFT 7
#define PIN_HALL_EFFECT_RIGHT 8
#define PIN_MOTOR_LEFT 9
#define PIN_MOTOR_RIGHT 3
#define PIN_RED 12
#define PIN_GREEN 11
#define PIN_BLUE 10

// Analog pins.
#define PIN_LASER_READER 0

// Configs.
// Number of wheel quarter-rotations to store in ticks array.
#define ROTATIONS_COLLECTED 10

// DEBUG ONLY TURN OFF MOTORS
#define RUN_MOTORS true

// Motor constants.
#define SERVO_CENTER 1520
// Temporarily gate range to safer numbers:
#define SERVO_FORWARD_MULTIPLIER 0.21
#define SERVO_REVERSE_MULTIPLIER 0.22
#define SERVO_FULL_REVERSE -520.0  // 1000-1520
#define SERVO_FULL_FORWARD 480.0   // 2000-1520

#define DEBUG_WRITE_SAMPLE_RATE 1000
#define CONTROLLER_SAMPLE_RATE 1

#define STAGE_ONE_TIME 24.58 * 1000

// Define State Machine.
enum PathState {
  START,
  CHECKPOINT_ONE,
  CHECKPOINT_TWO,
  BROOM_STICK_ABYSS,
  GET_ON_THE_LINE,
  EXIT_TO_THE_RAMP,
  DO_A_FLIP,
  FIND_THE_LINE,
  RED_LINE_RIDER,
  INVERTED_CUP_STRAIGHT,
  INVERTED_CUP_RIGHT,
  FINISH
};

typedef struct InfraredWrapper {
  int pin = -1;
  int isBlack = false;
  // The time this bit last detected flipped.
  uint32_t flippedAt = 0;
  bool reverse = false;
};

// Wheel data container.
typedef struct Wheel {
  // Motor controller.
  Servo* servo;
  int pinServo = -1;
  int pinHallEffect = -1;
  // Revolution collector:
  // The collection of timestamps of last revolutions.
  uint32_t ticks[ROTATIONS_COLLECTED];
  // The position to insert the next revolution.
  int index = 0;
  // The global revolutions for this motor since reset.
  int tickCount = 0;
};