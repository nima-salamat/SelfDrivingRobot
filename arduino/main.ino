#include <Servo.h>
#include <ctype.h>
#include <UltrasonicSensor.h>
#include "PulseQueue.h"

// Motor pins
int IN1 = 10;
int M1  = 12;
int IN2 = 11;
int M2  = 13;

// Servo
Servo myServo;
int SERVO_PIN = 9;

// Reed switch
const int REED_PIN = 2;
volatile int pulseCounter = 0;
volatile unsigned long lastPulseMicros = 0;
const unsigned long DEBOUNCE_US = 5000UL;

// Ultrasonic sensors
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

// Servo range
int servoCurrent = 90;
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;

// Motor state
bool movingByPulses = false;
int targetPulses = 0;
int currentMotorSpeed = 0;
int motorDirection = 1;

// Pulse queue
const int QUEUE_SIZE = 16;
PulseQueue queue(QUEUE_SIZE);

// Lane changing
bool lane_changing_mode = true;
char lane = 'R';
enum LaneState { LANE_NORMAL = 0, LANE_LEFT, LANE_RETURNING };
LaneState laneState = LANE_NORMAL;
unsigned long laneChangeStart = 0;

// Function declarations
void countPulse();
bool looksLikePulseGroups(const String &line);
void startMotorByPulses(char dirChar, int speedVal, int pulses, int servoAngle);
void startServoMoveImmediate(int target);
void stopMotor();
void startLaneChangeLeft();
void startReturnToRightLane();
void finishLaneReturn();

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(REED_PIN), countPulse, FALLING);

  ultraLeft.begin();
  ultraRight.begin();
  ultraSide.begin();

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
  queue.clear();
  queue.parseAndEnqueue(String("b 255 4 50 b 255 3 130"));
  laneState = LANE_LEFT;
  lane = 'L';
  laneChangeStart = millis();
}

void startReturnToRightLane() {
  if (laneState == LANE_RETURNING) return;
  queue.clear();
  queue.parseAndEnqueue(String("f 255 3 130 f 255 3 50"));
  laneState = LANE_RETURNING;
  laneChangeStart = millis();
}

void finishLaneReturn() {
  laneState = LANE_NORMAL;
  lane = 'R';
}

bool looksLikePulseGroups(const String &line) {
  if (line.length() == 0) return false;
  char c = tolower(line.charAt(0));
  return (c == 'f' || c == 'b');
}

void loop() {
  // Handle serial input
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
      int speedValue = work.substring(6).toInt();
      setMotorSpeed(speedValue);
    }
    else if (work.startsWith("servo ")) {
      int angle = work.substring(6).toInt();
      startServoMoveImmediate(angle);
    }
    else {
      queue.parseAndEnqueue(work);
    }

    inputString = "";
    stringComplete = false;
  }

  // Read sensors
  int distLeft  = ultraLeft.readCm();
  int distRight = ultraRight.readCm();
  int distSide  = !movingByPulses ? ultraSide.readCm() : 9999;

  if (lane == 'R') {
    if (distLeft <= STOP_DISTANCE_CM || distRight <= STOP_DISTANCE_CM) {
      if (!lane_changing_mode) {
        stopMotor();
        queue.clear();
        startServoMoveImmediate(90);
      } else if (laneState == LANE_NORMAL) {
        startLaneChangeLeft();
      }
    }
  }

  // Lane changing logic
  if (laneState == LANE_LEFT) {
    if (!movingByPulses && distSide <= SIDE_RETURN_DISTANCE_CM) {
      startReturnToRightLane();
    }
  } else if (laneState == LANE_RETURNING) {
    if (millis() - laneChangeStart >= 1000UL) {
      finishLaneReturn();
    }
  }

  // Queue execution
  if (!movingByPulses && queue.count() > 0) {
    PulseCmd next;
    if (queue.dequeue(next)) {
      startMotorByPulses(next.dir, next.speed, next.pulses, next.angle);
    }
  }

  // Stop motor if target pulses reached
  if (movingByPulses) {
    if (getPulseCount() >= targetPulses) {
      stopMotor();
      startServoMoveImmediate(90);

      if (laneState == LANE_LEFT && distSide <= SIDE_RETURN_DISTANCE_CM) {
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
      if (inputString.length() > 0) stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
