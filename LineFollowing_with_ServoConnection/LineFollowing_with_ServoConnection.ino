#include <Servo.h>
#include <SoftwareSerial.h>
SoftwareSerial espSerial(2, 13);

#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define MOTOR_SPEED 180

// Right motor
int enableRightMotor = 6;
int rightMotorPin1 = 7;
int rightMotorPin2 = 8;

// Left motor
int enableLeftMotor = 5;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

// Servo motors
Servo servo1; // Will go 60° each time, then reset to 0 after 180°
Servo servo2; // Will go to 60°, wait 5s, then go back
int servo1Pin = 3;
int servo2Pin = 4;

int servo1Angle = 0; // Start at 0°, increment by 60°, reset at 180°
bool isSpecialSequenceRunning = false;
unsigned long ignoreBlackStartTime = 0;
const unsigned long ignoreBlackDuration = 3000; // 3 seconds

void setup()
{
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  
  // Attach servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(servo1Angle); // Initial 0°
  servo2.write(0);

  rotateMotor(0, 0);
  Serial.begin(9600); // For ESP simulation
  espSerial.begin(9600);
}

void loop()
{
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  if (isSpecialSequenceRunning)
  {
    if (millis() - ignoreBlackStartTime < ignoreBlackDuration)
    {
      followLine(rightIRSensorValue, leftIRSensorValue, false);
    }
    else
    {
      isSpecialSequenceRunning = false;
    }
  }
  else
  {
    if (rightIRSensorValue == HIGH && leftIRSensorValue == HIGH)
    {
      rotateMotor(0, 0); // Stop robot
      delay(500);        // Short pause

      // === Servo1 Incremental Rotation ===
      servo1Angle += 60;
      if (servo1Angle > 180)
      {
        servo1Angle = 0;
      }
      servo1.write(servo1Angle);
      delay(1000);

      // === Servo2 one-time action ===
      servo2.write(60);
      delay(5000);
      servo2.write(0);

      sendCurrentTime();

      isSpecialSequenceRunning = true;
      ignoreBlackStartTime = millis();
    }
    else
    {
      followLine(rightIRSensorValue, leftIRSensorValue, true);
    }
  }
}

void followLine(int rightIR, int leftIR, bool stopOnBothBlack)
{
  if (rightIR == LOW && leftIR == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIR == HIGH && leftIR == LOW)
  {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIR == LOW && leftIR == HIGH)
  {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  else if (stopOnBothBlack && rightIR == HIGH && leftIR == HIGH)
  {
    rotateMotor(0, 0); // Stop
  }
  else
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED); // Default forward
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  // Right Motor
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  // Left Motor
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

void sendCurrentTime()
{
  espSerial.println("NOTIFY");
}