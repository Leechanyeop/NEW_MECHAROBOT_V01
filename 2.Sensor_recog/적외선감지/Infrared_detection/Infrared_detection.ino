// Pin mapping: adjust to your wiring
const uint8_t TRIG[6] = {2, 4, 6, 8, 10, 12};  // F-L, F-C, F-R, R-L, R-C, R-R
const uint8_t ECHO[6] = {3, 5, 7, 9, 11, 13};

const uint8_t N = 6;
const uint16_t MAX_US = 25000;     // ~4.3m timeout
const uint8_t SAMPLES = 3;
const uint16_t SAFE_CM = 25;       // threshold for avoidance (tune)
const uint16_t MIN_CM = 2;
const uint16_t MAX_CM = 300;

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    digitalWrite(TRIG[i], LOW);
  }
}

uint16_t measureOnce(uint8_t tPin, uint8_t ePin) {
  digitalWrite(tPin, LOW);
  delayMicroseconds(2);
  digitalWrite(tPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(tPin, LOW);

  unsigned long dur = pulseIn(ePin, HIGH, MAX_US);
  if (dur == 0) return 0; // timeout
  // distance in cm: (dur in us) * 0.034/2
  uint16_t cm = (uint16_t)(dur * 0.034 * 0.5);
  if (cm < MIN_CM || cm > MAX_CM) return 0;
  return cm;
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  // simple median without sorting heavy arrays
  if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
  if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
  return c;
}

uint16_t measureStable(uint8_t tPin, uint8_t ePin) {
  uint16_t a = measureOnce(tPin, ePin);
  delay(10);
  uint16_t b = measureOnce(tPin, ePin);
  delay(10);
  uint16_t c = measureOnce(tPin, ePin);
  return median3(a, b, c);
}

void loop() {
  uint16_t d[6] = {0};

  // sequential scan: front trio then rear trio
  for (uint8_t i = 0; i < N; i++) {
    d[i] = measureStable(TRIG[i], ECHO[i]);
    delay(25); // spacing to reduce crosstalk
  }

  // Labels: 0 F-L, 1 F-C, 2 F-R, 3 R-L, 4 R-C, 5 R-R
  Serial.print("FL:"); Serial.print(d[0]);
  Serial.print(" FC:"); Serial.print(d[1]);
  Serial.print(" FR:"); Serial.print(d[2]);
  Serial.print(" RL:"); Serial.print(d[3]);
  Serial.print(" RC:"); Serial.print(d[4]);
  Serial.print(" RR:"); Serial.println(d[5]);

  // Simple avoidance logic (replace with your motor control)
  if (d[1] > 0 && d[1] < SAFE_CM) {
    // Front center blocked -> choose clearer side
    if (d[0] == 0 && d[2] == 0) {
      // both unknown -> stop/scan
      stopRobot();
    } else if (d[0] > d[2]) {
      turnLeft();
    } else {
      turnRight();
    }
  } else if (isReversing() && d[4] > 0 && d[4] < SAFE_CM) {
    // Rear center blocked while reversing
    stopRobot();
  } else {
    forward();
  }

  delay(50);
}

// Stub motor functions (implement for your driver)
void forward()  { /* set motor speeds forward */ }
void stopRobot(){ /* stop motors */ }
void turnLeft() { /* left turn */ }
void turnRight(){ /* right turn */ }
bool isReversing() { return false; }