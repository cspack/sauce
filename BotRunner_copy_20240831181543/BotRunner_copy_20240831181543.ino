// Digital pins.
#define PIN_IR_LEFT 2
#define PIN_IR_CENTER 4
#define PIN_IR_RIGHT 5
#define PIN_HALL_EFFECT_LEFT 6
#define PIN_HALL_EFFECT_RIGHT 7
#define PIN_LASER_POWER 8
#define PIN_MOTOR_LEFT 3
#define PIN_MOTOR_RIGHT 9
// Analog pins.
#define PIN_LASER_READER 0

// Configs.
#define ROTATIONS_COLLECTED 100


uint64_t lastTime = 0;

struct Wheel {
  uint64_t ticks[ROTATIONS_COLLECTED];
  int index = 0;
  int tickCount = 0;

  void recordTick(void) {
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

void initWheel(Wheel wheel) {
  wheel.index = 0;
  wheel.tickCount = 0;
  for (int i = 0; i < sizeof(wheel.ticks); i++) wheel.ticks[i] = -1;
}
Wheel left;
Wheel right;

void leftInterrupt() {
  left.recordTick();
}

void rightInterrupt() {
  right.recordTick();
}

void setup() {
  initWheel(left);
  initWheel(right);

  Serial.begin(9600);
  Serial.println("Let's go!");

  // Set up the Hall effect sensor pin.
  pinMode(PIN_HALL_EFFECT_LEFT, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_LEFT), leftInterrupt, FALLING);
}

// AI told me this is how I print a uint64.
void printUint64(uint64_t value) {
  uint32_t high = value >> 32;
  uint32_t low = value & 0xFFFFFFFF;
  Serial.print(high, HEX);
  Serial.print(low, HEX);
}

void loop() {
  // Only give a status update every second.
  uint64_t now = millis();
  if ((now - lastTime) >= 1000) {
    Serial.print("[1s @");
    printUint64(now);
    Serial.print("] left ticks: ");
    //Serial.println(left.tickCount);
    Serial.print(left.countTicks(now, 1000));
    Serial.print(" right ticks: ");
    Serial.println(right.countTicks(now, 1000));
    lastTime = now;
  }
}
