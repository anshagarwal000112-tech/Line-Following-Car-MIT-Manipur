 5IR Line Follower Robot Using ESP32

This is a 5IR line follower robot that I made using an **ESP32**. It uses 5 digital IR sensors to detect the line and an **L298N motor driver** to control the two motors.

I designed, coded, assembled, and tested the robot myself. After testing and tuning the sensor values, the robot was able to follow the track, including curves, loops, and 90° turns.

---

Features

* ESP32 as the main controller
* 5 digital IR sensors
* L298N motor driver
* 300 RPM geared motors
* Custom sensor mapping
* Can follow curves and sharp turns
* Can handle loop tracks
* Batterypowered

---
Hardware Components

| Component          | Specification             |
| ------------------ | ------------------------- |
| Microcontroller    | ESP32                     |
| Sensors            | 5 × Digital IR Sensors    |
| Motor Driver       | L298N                     |
| Motors             | 2 × 300 RPM Geared Motors |
| Battery            | 2 × 2200mAh 3.7V Li-ion   |
| Voltage Regulation | Buck Converter            |
| Chassis            | Robot Chassis             |

---

 How It Works

The 5 IR sensors are placed at the front of the robot. They detect the difference between the black line and the surface around it.

The ESP32 reads the sensor values and decides which direction the robot needs to move. It then sends the signals to the L298N, which controls the two motors.

If the line moves towards one side, the robot adjusts its motors to bring itself back onto the line.

I also changed and tuned the sensor mapping while testing so that the robot could handle turns and curves better.

---

 Wiring

The wiring diagram for the robot is included in the `Electronics` folder.

It shows the connections between the ESP32, IR sensors, L298N, motors, and power system.

---

 Power System

The robot uses:

* 2 × 2200mAh 3.7V Li-ion batteries
* A buck converter

The buck converter is used to provide the required voltage for the electronics.

---

 Development Process

### Day 1

I started by planning the basic design of the robot and deciding where to place the 5 IR sensors.

After that, I worked on the code for reading the sensors and controlling the motors.

I connected the electronics and first tested the motors separately to make sure they were moving in the correct direction.

Then I tested the IR sensors and started testing the robot on a line track.

The first tests needed some changes to the sensor mapping and motor control. I kept testing and changing the values until the robot was able to follow the line properly.

---

 Testing

After finishing the basic setup, I tested the robot on different parts of the track.

It was able to handle:

* Line following
* Curves
* 90° turns
* Loop tracks

The robot needed some tuning for sharper turns, but after tuning the sensor values and motor control, it was able to complete the track reliably.

---

 What I Would Improve

If I make another version of this robot, I would like to improve the motor driver and make the wiring cleaner.

I would also like to make the robot lighter and spend more time tuning it for higher speed.

---

Final Result

This project was mainly about making a working line follower from the parts I had and getting it ready for testing.

After assembling everything, writing the code, and doing the tuning, the final robot was able to follow the track and handle curves, loops, and 90° turns.

<img width="675" height="900" alt="image" src="https://github.com/user-attachments/assets/c388d2d0-d45c-4e41-8f74-a1deaf96759f" />


