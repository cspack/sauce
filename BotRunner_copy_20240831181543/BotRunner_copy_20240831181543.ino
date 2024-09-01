// How many rotations to hold onto in the motor.
const int ROTATIONS_COLLECTED = 100;

struct Wheel {
  uint32_t ticks[ROTATIONS_COLLECTED];
  int index = 0;

  void recordTick(void) {
    int next = index++ % ROTATIONS_COLLECTED;
    ticks[next] = millis();
    index = next;
  }

  int countTicks(uint32_t time, int max_ms) {
    int count = 0;
    int i = index;
    int now = time;
    int min_time = now - max_ms;
    while (true) {
      int next = i--;
      // Modulus operator might make this simpler, but i'm not going to overthink.
      if (next < 0) next += ROTATIONS_COLLECTED;
      i = next;
      if (ticks[i] != -1 && ticks[i] >= min_time) {
        count++;
      } else {
        break;
      }
    }
    return count;
  }
};


static Wheel
initWheel() {
  Wheel wheel;
  for (int i = 0; i < sizeof(wheel.ticks); i++) wheel.ticks[i] = -1;
  return wheel;
}
Wheel left = initWheel();
Wheel right = initWheel();

void leftInterrupt() {
  left.recordTick();
}
void rightInterrupt() {
  right.recordTick();
}

void hallInterrupt() {
  Serial.println("BYE");
}

const int hallSensorPin = 3;

void setup() {
  Serial.begin(9600);

  // Set up the Hall effect sensor pin
  pinMode(hallSensorPin, INPUT);

  attachInterrupt(A1, leftInterrupt, FALLING);
  attachInterrupt(A2, rightInterrupt, FALLING);
  attachInterrupt(hallSensorPin, hallInterrupt, FALLING);

}

void loop() {
  uint32_t now = millis();
  Serial.print("[1s] left ticks: ");
  Serial.print(left.countTicks(now, 1000));
  Serial.print(" right ticks: ");
  Serial.println(right.countTicks(now, 1000));
}
