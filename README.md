# BotBrains_Battle_Round-2
Two-Wheeled Self-Balancing Robot with Autonomous Delivery Capabilities



Table of Contents
•	Project Overview
•	Assumptions
•	Components List
•	Circuitry
•	Explanation of Key Concepts
•	Conclusion


1. Project Overview
This project involves developing a Two-Wheeled Self-Balancing Robot (TWSBR) designed to autonomously deliver parcels to consumer doorsteps. The robot uses various sensors and actuators to balance, navigate traffic, handle turns, and avoid obstacles. The project is implemented using an Arduino Mega 2560 microcontroller, with necessary sensors and components integrated for autonomous operation.


2. Assumptions
1.	The robot operates in a predefined urban environment.
2.	GPS data is available for outdoor navigation.
3.	The robot has access to a digital map for path planning.
4.	The parcel dimensions and weight are within the robot's carrying capacity.
5.	The robot is equipped with necessary sensors for obstacle detection and navigation.

3. Components List
•	Arduino Mega 2560: Microcontroller to control the robot.
•	Motor Driver (L298N): To control the DC motors for movement.
•	DC Motors with Encoders: For precise movement and balancing.
•	Gyroscope and Accelerometer (MPU6050): For balancing.
•	Ultrasonic Sensors (HC-SR04): For obstacle detection.
•	GPS Module (NEO-6M): For outdoor navigation.
•	LIDAR Sensor (RPLIDAR A1): For detailed environment mapping and navigation.
•	Servo Motor: To adjust the wheelbase or any other mechanical adjustments.
•	Battery Pack: To power the components.
4. Circuitry
•	Arduino Mega 2560: Central microcontroller.
•	Motor Driver (L298N):
o	Connect motor outputs to DC motors.
o	Connect IN1, IN2, IN3, IN4 to Arduino digital pins for motor control.
o	Connect ENA and ENB to PWM pins for speed control.
•	MPU6050:
o	Connect VCC to 5V, GND to GND, SCL to A5, SDA to A4.
•	Ultrasonic Sensors:
o	Connect Trig and Echo pins to digital pins on the Arduino.
•	GPS Module:
o	Connect VCC to 3.3V, GND to GND, TX to RX, RX to TX on Arduino.
•	LIDAR Sensor:
o	Connect to Arduino via UART or dedicated interface.
•	Servo Motor:
o	Connect signal wire to PWM pin, VCC to 5V, GND to GND.

5. Explanation of Key Concepts
1.	Balancing the Robot: The MPU6050 sensor provides data on the robot's orientation. A PID controller is used to keep the robot balanced by adjusting the motor speeds.


Here is code:
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

MPU6050 mpu;
Servo myservo;
TinyGPSPlus gps;

// Motor pins
const int motor1Pin1 = 8;
const int motor1Pin2 = 9;
const int motor2Pin1 = 10;
const int motor2Pin2 = 11;
const int enableA = 5;
const int enableB = 6;

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 13;

// GPS module pins
SoftwareSerial gpsSerial(4, 3); // RX, TX

// PID constants
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float previousError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  gpsSerial.begin(9600);
  
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myservo.attach(7);
}

void loop() {
  // Balance control using MPU6050
  float angle = getAngle();
  float error = 0 - angle; // Target angle is 0 (upright)
  integral += error;
  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  setMotorSpeed(output);

  // Obstacle avoidance
  long distance = getDistance();
  if (distance < 20) {
    stopMotors();
    avoidObstacle();
  } else {
    moveForward();
  }

  // GPS navigation
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated()) {
    navigateTo(gps.location.lat(), gps.location.lng());
  }
}

float getAngle() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float angle = atan2(ay, az) * 180 / PI;
  return angle;
}

void setMotorSpeed(float speed) {
  int motorSpeed = constrain(speed, -255, 255);
  analogWrite(enableA, abs(motorSpeed));
  analogWrite(enableB, abs(motorSpeed));
  if (motorSpeed > 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void avoidObstacle() {
  myservo.write(90); // Turn the robot to avoid the obstacle
  delay(1000);
  myservo.write(0);  // Reset servo position
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void navigateTo(double targetLat, double targetLng) {
  double currentLat = gps.location.lat();
  double currentLng = gps.location.lng();
  double angleToTarget = atan2(targetLng - currentLng, targetLat - currentLat) * 180 / PI;
  // Adjust robot direction to move towards the target
  // Implement your navigation logic here
}




