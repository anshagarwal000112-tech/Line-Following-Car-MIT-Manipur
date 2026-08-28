# StarDance 5IR — Build Journal

## Why I made it

I wanted to build a fast line follower for competition. My first plan was to use a 16 IR sensor array, but that version did not work out because I damaged the sensor array.

The competition was getting close and I did not have another 16 IR sensor available. Instead of stopping the project, I decided to make a simpler 5 IR version with the parts I already had.

## Day 1 — Starting the 5IR version

I started by changing the design around the 5 IR sensors.

The main parts I used were an ESP32, 5 digital IR sensors, an L298N motor driver and two 300 RPM geared motors.

I first worked on the basic wiring and checked that the motors were connected correctly. I also tested the IR sensors separately to make sure I was getting the expected readings.

## Getting the sensors working

Once the basic electronics were connected, I worked on the sensor mapping.

The five sensors are placed in a row at the front of the robot. Their readings tell the ESP32 where the line is compared to the robot.

I had to test the sensors on the actual track because the readings were not something I wanted to just assume would work perfectly.

## Motor testing

After the sensor testing, I checked the two motors.

I made sure both motors were moving in the correct direction and then connected them through the L298N.

The 300 RPM motors gave the robot enough speed for what I wanted, but they also meant that the robot needed to make corrections quickly.

## First line-following tests

The first line tests were not perfect.

The robot could follow the line, but on sharper turns it sometimes went too far before correcting. I changed the sensor mapping and motor control and tested it again.

I repeated this process until the robot was able to stay on the line more consistently.

## Testing the track

After the basic line following was working, I tested different parts of the track instead of only testing a straight line.

I tested curves, 90-degree turns and loops.

The robot was able to handle these sections after tuning the sensor values and motor control.

## Final version

The final robot uses:

- ESP32
- 5 digital IR sensors
- L298N motor driver
- 2 × 300 RPM geared motors
- 2 × 2200mAh 3.7V Li-ion batteries
- Buck converter
- Robot chassis

The electronics, CAD files, wiring diagram and firmware are included in the repository.

## What I learned

The biggest thing I learned from this build was that a project does not always go according to the original plan.

I originally wanted to use 16 IR sensors, but after damaging that sensor array I had to change the design quickly. The 5 IR version was simpler, but it still worked well enough for the track I was testing on.

I also learned that sensor placement and tuning make a big difference in a line follower. The robot can have good motors and working code, but if the sensor readings and motor corrections are not tuned properly, it will still lose the line.

## What I would change next time

For another version, I would like to use a better motor driver, make the wiring cleaner, reduce the weight and try a larger IR sensor array again.

I would also spend more time testing and tuning before taking the robot to a competition.
