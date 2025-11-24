// Arduino sketch: queue multiple space-separated pulse commands on one line
// Example input (single-line):  f 255 4 80 b 255 8 110 f 255 2 90
// The parser accepts repeated groups: <F|B> <speed> <pulses> <angle>
// Also supports separators ';' or ',' if you prefer: F 200 5 120;B 150 3 90

#include <Servo.h>
#include <ctype.h>

// ----- Motor pins (unchanged)
int IN1 = 10;
int M1  = 12;
int IN2 = 11;
int M2  = 13;

// ----- Servo
Servo myServo;
int SERVO_PIN = 9;

// ----- Reed sensor for counting pulses (interrupt)
const int REED_PIN = 2;        // interrupt pin
volatile int pulseCounter = 0; // volatile because updated in ISR

// ----- Debounce
volatile unsigned long lastPulseMicros = 0; // last pulse time in microseconds
const unsigned long DEBOUNCE_US = 5000UL;   // 5ms debounce

// ----- Ultrasonic sensors (two)
const int ULTRA_LEFT_TRIG  = 4;
const int ULTRA_LEFT_ECHO  = 5;
const int ULTRA_RIGHT_TRIG = 6;
const int ULTRA_RIGHT_ECHO = 7;
const unsigned long ULTRA_TIMEOUT_US = 30000UL; // 30 ms -> ~5 meters
const int STOP_DISTANCE_CM = 20; // stop if any ultrasonic reads <= 20 cm

// ----- Debug
bool debugEnabled = true;
#undef DBG_PRINT
#undef DBG_PRINTLN
#define DBG_PRINT(x) ((void)0)
#define DBG_PRINTLN(x) ((void)0)

// ----- Serial input
String inputString = "";
bool stringComplete = false;

// ----- Servo movement state
int servoCurrent = 90;
int servoTarget = 90;
bool servoMoving = false;
int servoPresetIndex = 1;         // 0–2
int servoPresets[3] = {10, 5, 2}; // ms/degree
unsigned long servoLastStep = 0;
unsigned long servoStepDelay = 10;

// ----- Servo constraints
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

// ----- Motor-by-pulses control
bool movingByPulses = false;
int targetPulses = 0;
int currentMotorSpeed = 0;
int motorDirection = 1; // 1=forward, -1=backward

// ----- Queue for multiple pulse commands
const int QUEUE_SIZE = 16;
struct PulseCmd {
  char dir;      // 'f' or 'b'
  int speed;     // positive speed (0-255)
  int pulses;    // target pulses
  int angle;     // servo angle to set for this command
};
PulseCmd cmdQueue[QUEUE_SIZE];
int qHead = 0;
int qTail = 0;
int qCount = 0;

bool enqueuePulse(const PulseCmd &c) {
  if (qCount >= QUEUE_SIZE) return false;
  cmdQueue[qTail] = c;
  qTail = (qTail + 1) % QUEUE_SIZE;
  qCount++;
  return true;
}

bool dequeuePulse(PulseCmd &out) {
  if (qCount == 0) return false;
  out = cmdQueue[qHead];
  qHead = (qHead + 1) % QUEUE_SIZE;
  qCount--;
  return true;
}

void clearQueue() {
  qHead = qTail = qCount = 0;
}

// ----- Helper: get next whitespace/non-separator token from line
String getNextToken(const String &line, int &pos) {
  int len = line.length();
  // skip whitespace and separators (spaces, tabs, commas, semicolons)
  while (pos < len) {
    char c = line.charAt(pos);
    if (c == ' ' || c == '\t' || c == ',' || c == ';') pos++;
    else break;
  }
  if (pos >= len) return String("");
  int start = pos;
  while (pos < len) {
    char c = line.charAt(pos);
    if (c == ' ' || c == '\t' || c == ',' || c == ';') break;
    pos++;
  }
  return line.substring(start, pos);
}

// ----- Parse a line consisting of repeated groups: <F|B> <speed> <pulses> <angle> ...
void parseAndEnqueueSpaceSeparated(String line) {
  int pos = 0;
  int len = line.length();
  while (pos < len) {
    String t0 = getNextToken(line, pos);
    if (t0.length() == 0) break;

    // if token starts with 'f' or 'b' (or 'F'/'B'), treat as command head
    char head = tolower(t0.charAt(0));
    if (head != 'f' && head != 'b') {
      // If token isn't a direction, skip it (robustness)
      // Skipped unexpected token
      continue;
    }

    // read next three tokens: speed, pulses, angle
    String sSpeed = getNextToken(line, pos);
    String sPulses = getNextToken(line, pos);
    String sAngle = getNextToken(line, pos);

    if (sSpeed.length() == 0 || sPulses.length() == 0 || sAngle.length() == 0) {
      // Incomplete command after token, ignore remainder.
      break; // stop parsing further since group is incomplete
    }

    int speedVal = sSpeed.toInt();
    int pulses = sPulses.toInt();
    int angle = sAngle.toInt();

    if (pulses <= 0 || speedVal == 0) {
      // Invalid values in command: skip
      continue;
    }

    if (angle < SERVO_MIN) angle = SERVO_MIN;
    if (angle > SERVO_MAX) angle = SERVO_MAX;

    PulseCmd c;
    c.dir = head;
    c.speed = abs(speedVal);
    c.pulses = pulses;
    c.angle = angle;

    if (!enqueuePulse(c)) {
      // Queue full. Stop parsing and not enqueuing further commands.
      return;
    }

    // Enqueued -> no serial output (removed)
  }
}

// ----- Ultrasonic distance (cm)
int readUltrasonicCm(int trigPin, int echoPin) {
  // trigger a 10us pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0) return 9999; // timeout/no echo -> treat as far away

  // distance in cm: duration (us) / 58.2
  int distance = (int)(duration / 58.2);
  return distance;
}

// ----- Lane changing logic (uses pulse commands now)
bool lane_changing_mode = true;
char lane = 'R'; // 'L' or 'R'

// lane change state machine
enum LaneState { LANE_NORMAL = 0, LANE_LEFT, LANE_RETURNING };
LaneState laneState = LANE_NORMAL;
unsigned long laneChangeStart = 0;
const unsigned long LANE_LEFT_DURATION_MS = 6000UL;     // how long to stay on left lane before returning
const unsigned long LANE_RETURN_DURATION_MS = 3000UL;   // how long the return maneuver runs

// ----- Setup
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // on some boards this waits for serial attach

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(REED_PIN, INPUT_PULLUP); // reed sensor
  attachInterrupt(digitalPinToInterrupt(REED_PIN), countPulse, FALLING);

  // Ultrasonic pins
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);
  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);
  digitalWrite(ULTRA_LEFT_TRIG, LOW);
  digitalWrite(ULTRA_RIGHT_TRIG, LOW);

  myServo.attach(SERVO_PIN);
  myServo.write(servoCurrent);

  // Ready! (no serial output)
}

// ----- Interrupt for counting pulses with debounce
void countPulse() {
  unsigned long now = micros();
  if (now - lastPulseMicros >= DEBOUNCE_US) {
    pulseCounter++;
    lastPulseMicros = now;
  }
}

// ----- Safe read of pulseCounter
int getPulseCount() {
  noInterrupts();
  int cnt = pulseCounter;
  interrupts();
  return cnt;
}

// ----- Motor control
void setMotorSpeed(int speed) {
  int dir;
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed >= 0) dir = HIGH;
  else { dir = LOW; speed = -speed; }

  analogWrite(IN1, speed);
  analogWrite(IN2, speed);
  digitalWrite(M1, dir);
  digitalWrite(M2, dir);

  currentMotorSpeed = (dir == HIGH) ? speed : -speed;
}

void stopMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  movingByPulses = false;
  // Motor stopped (no serial output)
}

// ----- Servo movement helpers
unsigned long computeStepDelay(int delta) {
  int base = servoPresets[servoPresetIndex];
  if (delta > 25) return 0;
  else if (delta >= 15) return (unsigned long)base;
  else return (unsigned long)(base * 2UL);
}

void startServoMove(int target) {
  if (target < SERVO_MIN) target = SERVO_MIN;
  if (target > SERVO_MAX) target = SERVO_MAX;

  int delta = abs(target - servoCurrent);
  servoTarget = target;

  if (delta == 0) {
    servoMoving = false;
    return;
  }

  if (delta > 25) {
    myServo.write(servoTarget);
    servoCurrent = servoTarget;
    servoMoving = false;
    return;
  }

  servoStepDelay = computeStepDelay(delta);
  servoLastStep = millis();
  servoMoving = true;
}

// ----- Motor by pulses (core)
void startMotorByPulses(char dirChar, int speedVal, int pulses, int servoAngle) {
  noInterrupts();
  pulseCounter = 0;
  interrupts();

  targetPulses = pulses;
  movingByPulses = true;

  motorDirection = (tolower(dirChar) == 'f') ? 1 : -1;
  setMotorSpeed(speedVal * motorDirection);

  startServoMove(servoAngle);

  // START move (no serial output)
}

// ----- Lane-change helpers (non-blocking) now use pulse commands
void startLaneChangeLeft() {
  if (laneState != LANE_NORMAL) return; // already changing
  // clear existing queued commands, then enqueue a short pulse command to perform the lane change
  clearQueue();
  // enqueue: forward, full speed (255), 3 pulses, servo angle 130 (left steering)
  parseAndEnqueueSpaceSeparated(String("f 255 3 130")); // steering left for going to the left line
  laneState = LANE_LEFT;
  lane = 'L';
  laneChangeStart = millis();
  // Starting lane change (no serial output)
}

void startReturnToRightLane() {
  if (laneState == LANE_RETURNING) return;
  // clear any queued commands and enqueue the return pulse
  clearQueue();
  // enqueue: forward, full speed (255), 3 pulses, servo angle 60 (right steering / return)
  parseAndEnqueueSpaceSeparated(String("f 255 3 60")); // steering right for getting back to right line
  laneState = LANE_RETURNING;
  laneChangeStart = millis(); // start timer for the return stage if desired
  // Returning to right lane (no serial output)
}

void finishLaneReturn() {
  laneState = LANE_NORMAL;
  lane = 'R';
  // Optionally stop or set a safe speed
  stopMotor();
  // Lane return complete (no serial output)
}

// ----- Main loop
void loop() {
  // process serial input if a full line has arrived
  if (stringComplete) {
    inputString.trim();
    String work = inputString; // preserve original case for numeric tokens

    if (work.equalsIgnoreCase("stop")) {
      clearQueue();
      stopMotor();
      // Motors stopped, queue cleared (no serial output)
    }
    else if (work.startsWith("motor ")) {
      int speedValue = work.substring(6).toInt();
      setMotorSpeed(speedValue);
      // Motors set to speed (no serial output)
    }
    else if (work.startsWith("servo ")) {
      int angle = work.substring(6).toInt();
      if (angle < SERVO_MIN) angle = SERVO_MIN;
      if (angle > SERVO_MAX) angle = SERVO_MAX;
      startServoMove(angle);
      // Servo target angle set (no serial output)
    }
    else if (work.startsWith("servo_preset ")) {
      int idx = work.substring(13).toInt();
      if (idx < 0) idx = 0;
      if (idx > 2) idx = 2;
      servoPresetIndex = idx;
      // Servo preset index set (no serial output)
    }
    else if (work.startsWith("set_preset ")) {
      String rest = work.substring(11);
      int spacePos = rest.indexOf(' ');
      if (spacePos > 0) {
        int idx = rest.substring(0, spacePos).toInt();
        int val = rest.substring(spacePos + 1).toInt();
        if (idx >= 0 && idx <= 2 && val > 0) {
          servoPresets[idx] = val;
          // Preset updated (no serial output)
        }
      }
    }
    else {
      // try to parse as space-separated repeated pulse groups (preferred format)
      parseAndEnqueueSpaceSeparated(work);
    }

    inputString = "";
    stringComplete = false;
  }

  // ----- Ultrasonic checks (stop immediately if obstacle detected)
  int distLeft = readUltrasonicCm(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  int distRight = readUltrasonicCm(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);

  DBG_PRINT("Ultrasound L:");
  DBG_PRINT(distLeft);
  DBG_PRINT(" cm R:");
  DBG_PRINTLN(distRight);

  if (distLeft <= STOP_DISTANCE_CM || distRight <= STOP_DISTANCE_CM) {
    // obstacle detected
    if (!lane_changing_mode) {
      stopMotor();
      clearQueue();
      // center servo
      myServo.write(90);
      servoCurrent = 90;
      servoTarget = 90;
      servoMoving = false;
      // Obstacle detected -> stopped and queue cleared (lane changing disabled)
    } else {
      // lane changing enabled
      // only start lane change if we are currently in NORMAL state and on the right lane
      if (laneState == LANE_NORMAL && lane == 'R') {
        // start non-blocking lane change (now done by enqueuing a pulse)
        startLaneChangeLeft();
        // clear queued commands already done inside startLaneChangeLeft()
      } else {
        // already in lane change or already on left lane - do nothing extra
      }
    }
  }

  // ----- Lane-change timing handling (non-blocking)
  // Note: we still use timers to decide when to initiate return maneuver.
  if (laneState == LANE_LEFT) {
    if (millis() - laneChangeStart >= LANE_LEFT_DURATION_MS) {
      // time to return to right lane (enqueue return pulse)
      startReturnToRightLane();
    }
  } else if (laneState == LANE_RETURNING) {
    if (millis() - laneChangeStart >= LANE_RETURN_DURATION_MS) {
      finishLaneReturn();
    }
  }

  // ----- If not currently executing a pulse command but queue has items, start next
  if (!movingByPulses && qCount > 0) {
    PulseCmd next;
    if (dequeuePulse(next)) {
      startMotorByPulses(next.dir, next.speed, next.pulses, next.angle);
    }
  }

  // ----- Servo stepping (smooth moves)
  if (servoMoving) {
    unsigned long now = millis();
    if (now - servoLastStep >= servoStepDelay) {
      servoLastStep = now;
      if (servoCurrent < servoTarget) servoCurrent++;
      else if (servoCurrent > servoTarget) servoCurrent--;

      myServo.write(servoCurrent);

      if (servoCurrent == servoTarget) {
        servoMoving = false;
        // Servo reached target (no serial output)
      }
    }
  }

  // ----- Motor by pulses progress
  if (movingByPulses) {
    int pc = getPulseCount();

    if (pc >= targetPulses) {
      stopMotor();

      // Safe reset of servo to 90 with internal variables
      myServo.write(90);
      servoCurrent = 90;
      servoTarget = 90;
      servoMoving = false;

      movingByPulses = false; // next iteration will start next queued command (if any)

      // Target pulses reached (no serial output)
    } else {
      DBG_PRINT("Pulse progress: ");
      DBG_PRINT(pc);
      DBG_PRINT("/");
      DBG_PRINTLN(targetPulses);
    }
  }

  // small sleep to avoid tight loop (keeps responsiveness)
  delay(1);
}

// ----- Serial event (line buffering)
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0)
        stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
