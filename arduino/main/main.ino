#include <Servo.h>
// ----------- Pin Definitions -----------
// L298P Motor A (Left Motor)
const int MotorA1 = 12; // IN1
const int MotorA2 = 3;  // IN2
const int APWM = 10;    // ENA (PWM control)

// L298P Motor B (Right Motor)
const int MotorB1 = 13; // IN3
const int MotorB2 = 8;  // IN4
const int BPWM = 11;    // ENB (PWM control)

// Servo and Ultrasonic Sensor
const int servoPin = 9;
const int ultrasonicTrig = 7;
const int ultrasonicEcho = 6;

// Control Parameters
int normalSpeed = 130; // Default speed
const int OBSTACLE_THRESHOLD = 15;

// Servo angle mapping
const int ANGLE_SHARP_LEFT = 0;
const int ANGLE_MID_LEFT = 40;
const int ANGLE_SLOW_LEFT = 55;
const int ANGLE_CENTER = 70;
const int ANGLE_SLOW_RIGHT = 85;
const int ANGLE_MID_RIGHT = 100;
const int ANGLE_SHARP_RIGHT = 140;

// Global Variables
bool ultrasonicEnabled = false;
String receivedCommand = "";
String previousCommand = "";
Servo myServo;
int lastServoAngle = ANGLE_CENTER; // Keeps track of the last servo angle

void setup()
{
  Serial.begin(9600);
  Serial.println("Arduino Lane Follower started with L298P Motor Shield.");
  Serial.println("Expected commands: 'steering speed' (e.g., 'sharp left 125'), 'ultrasonic on/off'");

  // Setup Motor Pins
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(APWM, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  pinMode(BPWM, OUTPUT);

  // Setup Ultrasonic Sensor
  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);

  // Attach Servo and Set to Center
  myServo.attach(servoPin);
  myServo.write(ANGLE_CENTER);
  lastServoAngle = ANGLE_CENTER;
}

// Function to measure distance using the ultrasonic sensor (in cm)
long measureDistance()
{
  digitalWrite(ultrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);

  long duration = pulseIn(ultrasonicEcho, HIGH, 30000);
  return duration * 0.034 / 2;
}

// Motor Control Function
void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  if (leftSpeed > 0)
  {
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, HIGH);
  }
  else if (leftSpeed < 0)
  {
    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);
  }
  else
  {
    digitalWrite(MotorA1, 0);
    digitalWrite(MotorA2, 0);
  }

  if (rightSpeed > 0)
  {
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, HIGH);
  }
  else if (rightSpeed < 0)
  {
    digitalWrite(MotorB1, HIGH);
    digitalWrite(MotorB2, LOW);
  }
  else
  {
    digitalWrite(MotorB1, 0);
    digitalWrite(MotorB2, 0);
  }

  analogWrite(APWM, abs(leftSpeed));
  analogWrite(BPWM, abs(rightSpeed));
}

void loop()
{
  // Read Serial Command
  if (Serial.available() > 0)
  {
    receivedCommand = Serial.readStringUntil('\n');
    receivedCommand.trim();

    if (receivedCommand.equalsIgnoreCase("ultrasonic on"))
    {
      ultrasonicEnabled = true;
      Serial.println("Ultrasonic enabled.");
    }
    else if (receivedCommand.equalsIgnoreCase("ultrasonic off"))
    {
      ultrasonicEnabled = false;
      Serial.println("Ultrasonic disabled.");
    }
    else
    {
      // Parse the command for steering and speed
      int spaceIndex = receivedCommand.lastIndexOf(' ');
      if (spaceIndex != -1)
      {
        String steeringCommand = receivedCommand.substring(0, spaceIndex);
        String speedStr = receivedCommand.substring(spaceIndex + 1);
        int newSpeed = speedStr.toInt();

        // Validate speed
        if (newSpeed >= 0 && newSpeed <= 255)
        {
          normalSpeed = newSpeed;
          Serial.print("Received command: ");
          Serial.print(steeringCommand);
          Serial.print(" | Speed set to: ");
          Serial.println(normalSpeed);

          // Determine Servo Angle based on steering command
          int servoAngle = lastServoAngle; // Start from the last known angle
          if (receivedCommand.equalsIgnoreCase("stop")) {
            // Stop the motors immediately and optionally update the servo.
            setMotorSpeed(0, 0);
            Serial.println("Motors stopped.");
            return; // Skip the rest of the loop iteration
          }

          if (steeringCommand.equalsIgnoreCase("sharp left"))
          {
            servoAngle = ANGLE_SHARP_LEFT;
          }
          else if (steeringCommand.equalsIgnoreCase("mid left"))
          {
            servoAngle = ANGLE_MID_LEFT;
          }
          else if (steeringCommand.equalsIgnoreCase("slow left"))
          {
            servoAngle = ANGLE_SLOW_LEFT;
          }
          else if (steeringCommand.equalsIgnoreCase("sharp right"))
          {
            servoAngle = ANGLE_SHARP_RIGHT;
          }
          else if (steeringCommand.equalsIgnoreCase("mid right"))
          {
            servoAngle = ANGLE_MID_RIGHT;
          }
          else if (steeringCommand.equalsIgnoreCase("slow right"))
          {
            servoAngle = ANGLE_SLOW_RIGHT;
          }
          else if (steeringCommand.equalsIgnoreCase("center"))
          {
            // Adjust center based on the last servo position
            if (lastServoAngle == ANGLE_SHARP_RIGHT)
            {
              servoAngle = ANGLE_CENTER - 10; // Move 10° to the right
            }
            else if (lastServoAngle == ANGLE_SHARP_LEFT)
            {
              servoAngle = ANGLE_CENTER + 10; // Move 10° to the left
            }
            else
            {
              servoAngle = ANGLE_CENTER;
            }
          }

          // Write the new servo angle and update the last known angle
          myServo.write(servoAngle);
          lastServoAngle = servoAngle;
        }
        else
        {
          Serial.println("Invalid speed value. Use a number between 0 and 255.");
        }
      }
      else
      {
        Serial.println("Invalid command format. Expected: 'steering speed' (e.g., 'sharp left 125')");
      }
    }

    // Save the command and clear it
    if (receivedCommand != "")
    {
      previousCommand = receivedCommand;
      receivedCommand = "";
    }
  }

  // Motor Speed Control
  int motorSpeed = normalSpeed;
  if (ultrasonicEnabled)
  {
    long distance = measureDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > 0 && distance < OBSTACLE_THRESHOLD)
    {
      motorSpeed = 0;
      Serial.println("Obstacle detected! Motor stopped.");
    }
  }

  // Apply Motor Speed to both motors
  setMotorSpeed(motorSpeed, motorSpeed);

  delay(50);
}