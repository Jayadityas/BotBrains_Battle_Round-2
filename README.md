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

