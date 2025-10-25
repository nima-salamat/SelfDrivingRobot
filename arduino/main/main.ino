#include <Servo.h>

// ----- Motor pins
int IN1 = 10; 
int M1 = 12;

int IN2 = 11;
int M2 = 13;

// ----- Servo
Servo myServo;
int SERVO_PIN = 9;

// ----- Serial input
String inputString = "";       
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  myServo.attach(SERVO_PIN);

  Serial.println("Ready! Commands: 'motor <speed>', 'servo <angle>', 'stop'");
}

// ----- Set speed for both motors
void setMotorSpeed(int speed) {
  int dir;
  
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed >= 0) {
    dir = HIGH;
  } else {
    dir = LOW;
    speed = -speed;
  }

  // Apply same speed and direction to both motors
  analogWrite(IN1, speed);
  analogWrite(IN2, speed);

  digitalWrite(M1, dir);
  digitalWrite(M2, dir);
}

// ----- Stop motors
void stopMotor() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}

// ----- Loop
void loop() {
  if (stringComplete) {
    inputString.trim();
    inputString.toLowerCase();

    if (inputString == "stop") {
      stopMotor();
      Serial.println("Motors stopped");
    } 
    else if (inputString.startsWith("motor ")) {
      int speedValue = inputString.substring(6).toInt();
      setMotorSpeed(speedValue);
      Serial.print("Both motors set to speed: ");
      Serial.println(speedValue);
    } 
    else if (inputString.startsWith("servo ")) {
      int angle = inputString.substring(6).toInt();
      if (angle < 0) angle = 0;
      if (angle > 180) angle = 180;
      myServo.write(angle);
      Serial.print("Servo set to angle: ");
      Serial.println(angle);
    } 
    else {
      Serial.println("Unknown command!");
    }

    inputString = "";
    stringComplete = false;
  }
}

// ----- Serial input
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
