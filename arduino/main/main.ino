// Arduino sketch: motor + steering servo controller via serial
// Requires: Adafruit Motor Shield v2 library and Servo library
// Install Adafruit_MotorShield library (Adafruit Motor Shield v2) in Arduino IDE

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // default I2C addr 0x60
Adafruit_DCMotor *leftMotor = NULL;   // will use M1
Adafruit_DCMotor *rightMotor = NULL;  // will use M2

Servo steeringServo;
const int STEERING_PIN = 9; // change if needed

// defaults
int motorSpeed = 180; // 0..255 (default forward speed)
int steeringAngle = 90; // 0..180 (center ~90)

String inputLine = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for serial on some boards

  Serial.println("Arduino motor+servo controller starting...");

  // start motor shield
  if (!AFMS.begin()) {
    Serial.println("ERR: Motor shield not found");
    while (1) delay(10);
  }
  leftMotor = AFMS.getMotor(1);  // M1
  rightMotor = AFMS.getMotor(2); // M2

  // attach servo
  steeringServo.attach(STEERING_PIN);
  steeringServo.write(steeringAngle);

  // ensure motors stopped
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  Serial.println("OK: Ready");
  printStatus();
}

void loop() {
  // read serial line (terminated by \\n or \\r)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\\n' || c == '\\r') {
      if (inputLine.length() > 0) {
        processCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
      // safety: limit line length
      if (inputLine.length() > 64) inputLine = inputLine.substring(inputLine.length() - 64);
    }
  }

  // optional: here you could implement periodic tasks (battery, sensors...)
  delay(2);
}

void processCommand(String cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.length() == 0) {
    Serial.println("ERR: empty");
    return;
  }

  // STOP (word)
  if (cmd == "STOP") {
    stopMotors();
    Serial.println("OK: STOP");
    return;
  }

  // Single-letter commands
  if (cmd == "F") {
    driveForward();
    Serial.println("OK: FWD");
    return;
  }
  if (cmd == "B") {
    driveBackward();
    Serial.println("OK: BACK");
    return;
  }

  // S<number> => steering servo angle
  if (cmd.charAt(0) == 'S') {
    String num = cmd.substring(1);
    if (num.length() == 0) {
      Serial.println("ERR: S no value");
      return;
    }
    int angle = num.toInt();
    angle = constrain(angle, 0, 180);
    steeringAngle = angle;
    steeringServo.write(steeringAngle);
    Serial.print("OK: S=");
    Serial.println(steeringAngle);
    return;
  }

  // M<number> => motor speed 0..255
  if (cmd.charAt(0) == 'M') {
    String num = cmd.substring(1);
    if (num.length() == 0) {
      Serial.println("ERR: M no value");
      return;
    }
    int sp = num.toInt();
    if (sp < 0) sp = 0;
    if (sp > 255) sp = 255;
    motorSpeed = sp;
    Serial.print("OK: M=");
    Serial.println(motorSpeed);
    return;
  }

  // Optionally, accept numeric-only as steering (for backward compatibility)
  bool numericOnly = true;
  for (unsigned int i = 0; i < cmd.length(); ++i) {
    if (!isDigit(cmd.charAt(i))) { numericOnly = false; break; }
  }
  if (numericOnly) {
    int val = cmd.toInt();
    // decide: if in 0..180 treat as steering
    if (val >= 0 && val <= 180) {
      steeringAngle = val;
      steeringServo.write(steeringAngle);
      Serial.print("OK: S(numeric)=");
      Serial.println(steeringAngle);
      return;
    }
  }

  Serial.print("ERR: unknown cmd '");
  Serial.print(cmdRaw);
  Serial.println("'");
}

void driveForward() {
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void driveBackward() {
  leftMotor->setSpeed(motorSpeed);
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void stopMotors() {
  // brake both motors
  leftMotor->run(BRAKE);
  rightMotor->run(BRAKE);
}

void printStatus() {
  Serial.print("Status - motorSpeed=");
  Serial.print(motorSpeed);
  Serial.print(" steeringAngle=");
  Serial.println(steeringAngle);
}
