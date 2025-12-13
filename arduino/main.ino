#include <Servo.h>
#include <ctype.h>
#include <UltrasonicSensor.h>
#include <PulseQueue.h>
#include "Encoder.h"

/* =========================
   MOTOR PINS
   ========================= */
int IN1 = 10;
int M1  = 12;
int IN2 = 11;
int M2  = 13;

/* =========================
   SERVO
   ========================= */
Servo myServo;
int SERVO_PIN = 9;
int servoCurrent = 90;
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

/* =========================
   ENCODER
   ========================= */
const int REED_PIN = 2;
const unsigned long DEBOUNCE_US = 5000UL;   // HALL: 5000, LASER: 0
Encoder encoder(REED_PIN, DEBOUNCE_US, FALLING);

/* =========================
   ULTRASONIC SENSORS
   ========================= */
const int ULTRA_LEFT_TRIG  = 4;
const int ULTRA_LEFT_ECHO  = 5;
const int ULTRA_RIGHT_TRIG = 6;
const int ULTRA_RIGHT_ECHO = 7;
const int ULTRA_SIDE_TRIG  = 8;
const int ULTRA_SIDE_ECHO  = 24;

const unsigned long ULTRA_TIMEOUT_US = 30000UL;
const int STOP_DISTANCE_CM = 35;
const int SIDE_RETURN_DISTANCE_CM = 20;

UltrasonicSensor ultraLeft(ULTRA_LEFT_TRIG,  ULTRA_LEFT_ECHO,  ULTRA_TIMEOUT_US);
UltrasonicSensor ultraRight(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO, ULTRA_TIMEOUT_US);
UltrasonicSensor ultraSide(ULTRA_SIDE_TRIG,   ULTRA_SIDE_ECHO,  ULTRA_TIMEOUT_US);

/* =========================
   MOTOR STATE
   ========================= */
bool movingByPulses = false;
int targetPulses = 0;
int currentMotorSpeed = 0;
int motorDirection = 1;

/* =========================
   PULSE QUEUE
   ========================= */
PulseQueue queue(16);

/* =========================
   LANE LOGIC
   ========================= */
bool lane_changing_mode = true;
char lane = 'R';

enum LaneState {
  LANE_NORMAL = 0,
  LANE_LEFT,
  LANE_RETURNING
};

LaneState laneState = LANE_NORMAL;
unsigned long laneChangeStart = 0;

/* =========================
   SERIAL INPUT
   ========================= */
String inputString = "";
bool stringComplete = false;

/* =========================
   TELEMETRY
   ========================= */
unsigned long lastStatusSend = 0;
const unsigned long STATUS_INTERVAL_MS = 30;

/* =========================
   FUNCTION DECLARATIONS
   ========================= */
void stopMotor();
void startServoMoveImmediate(int target);
void startMotorByPulses(char dirChar, int speedVal, int pulses, int servoAngle);
void startLaneChangeLeft();
void startReturnToRightLane();
void finishLaneReturn();
bool looksLikePulseGroups(const String &line);

/* =========================
   SETUP
   ========================= */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  encoder.begin();

  ultraLeft.begin();
  ultraRight.begin();
  ultraSide.begin();

  myServo.attach(SERVO_PIN);
  myServo.write(servoCurrent);
}

/* =========================
   HELPERS
   ========================= */
int getPulseCount() {
  return encoder.getCount();
}

void resetPulseCount() {
  encoder.reset();
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
  currentMotorSpeed = 0;
}

void startServoMoveImmediate(int target) {
  if (target < SERVO_MIN) target = SERVO_MIN;
  if (target > SERVO_MAX) target = SERVO_MAX;
  myServo.write(target);
  servoCurrent = target;
}

void startMotorByPulses(char dirChar, int speedVal, int pulses, int servoAngle) {
  noInterrupts();
  resetPulseCount();
  interrupts();

  targetPulses = pulses;
  movingByPulses = true;
  motorDirection = (tolower(dirChar) == 'f') ? 1 : -1;

  setMotorSpeed(speedVal * motorDirection);
  startServoMoveImmediate(servoAngle);
}

/* =========================
   LANE FUNCTIONS
   ========================= */
void startLaneChangeLeft() {
  if (laneState != LANE_NORMAL) return;
  queue.clear();
  queue.parseAndEnqueue("b 255 4 50 b 255 3 130");
  laneState = LANE_LEFT;
  lane = 'L';
  laneChangeStart = millis();
}

void startReturnToRightLane() {
  if (laneState == LANE_RETURNING) return;
  queue.clear();
  queue.parseAndEnqueue("f 255 3 130 f 255 3 50");
  laneState = LANE_RETURNING;
  laneChangeStart = millis();
}

void finishLaneReturn() {
  laneState = LANE_NORMAL;
  lane = 'R';
}

/* =========================
   TELEMETRY HELPERS
   ========================= */
char motionChar() {
  if (currentMotorSpeed == 0) return 'S';
  if (currentMotorSpeed > 0) return 'F';
  return 'B';
}

bool obstacleFront() {
  int dl = ultraLeft.readCm();
  int dr = ultraRight.readCm();
  return (dl <= STOP_DISTANCE_CM || dr <= STOP_DISTANCE_CM);
}

void sendStatus() {
  Serial.print('<');
  Serial.print(lane);              // R / L
  Serial.print(',');
  Serial.print(motionChar());      // S / F / B
  Serial.print(',');
  Serial.print(obstacleFront() ? '1' : '0');
  Serial.println('>');
}

/* =========================
   MAIN LOOP
   ========================= */
void loop() {
  if (stringComplete) {
    inputString.trim();
    String work = inputString;

    if (movingByPulses && looksLikePulseGroups(work)) {
      queue.parseAndEnqueue(work);
    }
    else if (work.equalsIgnoreCase("stop")) {
      queue.clear();
      stopMotor();
    }
    else if (work.startsWith("motor ")) {
      setMotorSpeed(work.substring(6).toInt());
    }
    else if (work.startsWith("servo ")) {
      startServoMoveImmediate(work.substring(6).toInt());
    }
    else {
      queue.parseAndEnqueue(work);
    }

    inputString = "";
    stringComplete = false;
  }

  int distSide = (!movingByPulses) ? ultraSide.readCm() : 9999;

  if (lane == 'R' && obstacleFront()) {
    if (!lane_changing_mode) {
      stopMotor();
      queue.clear();
      startServoMoveImmediate(90);
    } else if (laneState == LANE_NORMAL) {
      startLaneChangeLeft();
    }
  }

  if (laneState == LANE_LEFT && !movingByPulses && distSide <= SIDE_RETURN_DISTANCE_CM) {
    startReturnToRightLane();
  }

  if (laneState == LANE_RETURNING && millis() - laneChangeStart >= 1000UL) {
    finishLaneReturn();
  }

  if (!movingByPulses && queue.count() > 0) {
    PulseCmd cmd;
    if (queue.dequeue(cmd)) {
      startMotorByPulses(cmd.dir, cmd.speed, cmd.pulses, cmd.angle);
    }
  }

  if (movingByPulses && getPulseCount() >= targetPulses) {
    stopMotor();
    startServoMoveImmediate(90);
  }

  if (millis() - lastStatusSend >= STATUS_INTERVAL_MS) {
    lastStatusSend = millis();
    sendStatus();
  }

  delay(1);
}

/* =========================
   SERIAL EVENT
   ========================= */
void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length()) stringComplete = true;
    } else {
      inputString += c;
    }
  }
}

/* =========================
   PARSER CHECK
   ========================= */
bool looksLikePulseGroups(const String &line) {
  if (line.length() == 0) return false;
  char c = tolower(line.charAt(0));
  return (c == 'f' || c == 'b');
}
