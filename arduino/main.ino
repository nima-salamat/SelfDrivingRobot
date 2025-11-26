#include <Servo.h>
#include <ctype.h>

int IN1 = 10;
int M1  = 12;
int IN2 = 11;
int M2  = 13;

Servo myServo;
int SERVO_PIN = 9;

const int REED_PIN = 2;
volatile int pulseCounter = 0;

volatile unsigned long lastPulseMicros = 0;
const unsigned long DEBOUNCE_US = 5000UL;

const int ULTRA_LEFT_TRIG  = 4;
const int ULTRA_LEFT_ECHO  = 5;
const int ULTRA_RIGHT_TRIG = 6;
const int ULTRA_RIGHT_ECHO = 7;
const int ULTRA_SIDE_TRIG = 8;
const int ULTRA_SIDE_ECHO = 24;
const unsigned long ULTRA_TIMEOUT_US = 30000UL;
const int STOP_DISTANCE_CM = 35;
const int SIDE_RETURN_DISTANCE_CM = 20;

bool debugEnabled = true;
#undef DBG_PRINT
#undef DBG_PRINTLN
#define DBG_PRINT(x) ((void)0)
#define DBG_PRINTLN(x) ((void)0)

String inputString = "";
bool stringComplete = false;

int servoCurrent = 90;
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

bool movingByPulses = false;
int targetPulses = 0;
int currentMotorSpeed = 0;
int motorDirection = 1;

const int QUEUE_SIZE = 16;
struct PulseCmd {
  char dir;
  int speed;
  int pulses;
  int angle;
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

String getNextToken(const String &line, int &pos) {
  int len = line.length();
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

void parseAndEnqueueSpaceSeparated(String line) {
  int pos = 0;
  int len = line.length();
  while (pos < len) {
    String t0 = getNextToken(line, pos);
    if (t0.length() == 0) break;
    char head = tolower(t0.charAt(0));
    if (head != 'f' && head != 'b') {
      continue;
    }
    String sSpeed = getNextToken(line, pos);
    String sPulses = getNextToken(line, pos);
    String sAngle = getNextToken(line, pos);
    if (sSpeed.length() == 0 || sPulses.length() == 0 || sAngle.length() == 0) {
      break;
    }
    int speedVal = sSpeed.toInt();
    int pulses = sPulses.toInt();
    int angle = sAngle.toInt();
    if (pulses <= 0 || speedVal == 0) {
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
      return;
    }
  }
}

int readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0) return 9999;
  int distance = (int)(duration / 58.2);
  return distance;
}

bool lane_changing_mode = true;
char lane = 'R';

enum LaneState { LANE_NORMAL = 0, LANE_LEFT, LANE_RETURNING };
LaneState laneState = LANE_NORMAL;
unsigned long laneChangeStart = 0;

void countPulse();
bool looksLikePulseGroups(const String &line);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), countPulse, FALLING);
  pinMode(ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(ULTRA_LEFT_ECHO, INPUT);
  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);
  pinMode(ULTRA_SIDE_TRIG, OUTPUT);
  pinMode(ULTRA_SIDE_ECHO, INPUT);
  digitalWrite(ULTRA_LEFT_TRIG, LOW);
  digitalWrite(ULTRA_RIGHT_TRIG, LOW);
  digitalWrite(ULTRA_SIDE_TRIG, LOW);
  myServo.attach(SERVO_PIN);
  myServo.write(servoCurrent);
}

void countPulse() {
  unsigned long now = micros();
  if (now - lastPulseMicros >= DEBOUNCE_US) {
    pulseCounter++;
    lastPulseMicros = now;
  }
}

int getPulseCount() {
  noInterrupts();
  int cnt = pulseCounter;
  interrupts();
  return cnt;
}

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
}

void startServoMoveImmediate(int target) {
  if (target < SERVO_MIN) target = SERVO_MIN;
  if (target > SERVO_MAX) target = SERVO_MAX;
  myServo.write(target);
  servoCurrent = target;
}

void startMotorByPulses(char dirChar, int speedVal, int pulses, int servoAngle) {
  noInterrupts();
  pulseCounter = 0;
  interrupts();
  targetPulses = pulses;
  movingByPulses = true;
  motorDirection = (tolower(dirChar) == 'f') ? 1 : -1;
  setMotorSpeed(speedVal * motorDirection);
  startServoMoveImmediate(servoAngle);
}

void startLaneChangeLeft() {
  if (laneState != LANE_NORMAL) return;
  clearQueue();
  parseAndEnqueueSpaceSeparated(String("b 255 4 50 b 255 4 130"));
  laneState = LANE_LEFT;
  lane = 'L';
  laneChangeStart = millis();
}

void startReturnToRightLane() {
  if (laneState == LANE_RETURNING) return;
  clearQueue();
  parseAndEnqueueSpaceSeparated(String("f 255 3 130 f 255 3 50"));
  laneState = LANE_RETURNING;
  laneChangeStart = millis();
}

void finishLaneReturn() {
  laneState = LANE_NORMAL;
  lane = 'R';
}

bool looksLikePulseGroups(const String &line) {
  int pos = 0;
  String t = getNextToken(line, pos);
  if (t.length() == 0) return false;
  char c = tolower(t.charAt(0));
  return (c == 'f' || c == 'b');
}

void loop() {
  if (stringComplete) {
    inputString.trim();
    String work = inputString;
    if (movingByPulses) {
      if (looksLikePulseGroups(work)) {
        parseAndEnqueueSpaceSeparated(work);
      }
      inputString = "";
      stringComplete = false;
      goto after_serial_processing;
    }
    if (work.equalsIgnoreCase("stop")) {
      clearQueue();
      stopMotor();
    }
    else if (work.startsWith("motor ")) {
      int speedValue = work.substring(6).toInt();
      setMotorSpeed(speedValue);
    }
    else if (work.startsWith("servo ")) {
      int angle = work.substring(6).toInt();
      if (angle < SERVO_MIN) angle = SERVO_MIN;
      if (angle > SERVO_MAX) angle = SERVO_MAX;
      startServoMoveImmediate(angle);
    }
    else {
      parseAndEnqueueSpaceSeparated(work);
    }
    inputString = "";
    stringComplete = false;
  }

after_serial_processing:

  int distLeft = readUltrasonicCm(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO);
  int distRight = readUltrasonicCm(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
  int distSide = 9999;

  if (!movingByPulses) {
    distSide = readUltrasonicCm(ULTRA_SIDE_TRIG, ULTRA_SIDE_ECHO);
    Serial.print("SIDE: ");
    Serial.println(distSide);
  }

  if (lane == 'R') {
    if (distLeft <= STOP_DISTANCE_CM || distRight <= STOP_DISTANCE_CM) {
      if (!lane_changing_mode) {
        stopMotor();
        clearQueue();
        myServo.write(90);
        servoCurrent = 90;
      } else {
        if (laneState == LANE_NORMAL && lane == 'R') {
          startLaneChangeLeft();
        }
      }
    }
  }

  if (laneState == LANE_LEFT) {
    if (!movingByPulses && distSide <= SIDE_RETURN_DISTANCE_CM) {
      startReturnToRightLane();
    }
  } else if (laneState == LANE_RETURNING) {
    if (millis() - laneChangeStart >= 10000UL) {
      finishLaneReturn();
    }
  }

  if (!movingByPulses && qCount > 0) {
    PulseCmd next;
    if (dequeuePulse(next)) {
      startMotorByPulses(next.dir, next.speed, next.pulses, next.angle);
    }
  }

  if (movingByPulses) {
    int pc = getPulseCount();
    if (pc >= targetPulses) {
      stopMotor();
      myServo.write(90);
      servoCurrent = 90;
      movingByPulses = false;
      int distSideAfter = readUltrasonicCm(ULTRA_SIDE_TRIG, ULTRA_SIDE_ECHO);
      Serial.print("SIDE: ");
      Serial.println(distSideAfter);
      if (laneState == LANE_LEFT && distSideAfter <= SIDE_RETURN_DISTANCE_CM) {
        startReturnToRightLane();
      }
    }
  }

  delay(1);
}

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
