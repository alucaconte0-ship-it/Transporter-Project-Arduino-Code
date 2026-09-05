# Autonomous Transporter Robot

First-year Mechanical Engineering group project at Brunel University London.

## Objective

Design and build an autonomous robot capable of collecting a LEGO block,
locating a randomly positioned target within a 1 m × 1 m arena, transporting
the block and placing it onto the target without manual control.

## My Contribution

- Developed the concept selected as the basis for the final robot
- Produced the CAD model and BS 8888 technical drawings in Autodesk Fusion
- Wrote the complete Arduino control software
- Developed autonomous gripping, lifting, scanning, target detection,
  approach, placement and retreat logic
- Iteratively tuned the software through physical testing
- Contributed to physical construction and system integration

## Control System

The robot uses a Makeblock MegaPi controller and ultrasonic sensor.

The software operates as a state machine:

1. Grip payload
2. Raise gripper
3. Scan for target
4. Reposition if target is not detected
5. Approach target
6. Slow for final positioning
7. Place payload
8. Retreat

## Hardware

- Makeblock MegaPi
- Ultrasonic distance sensor
- 2 × drive motors
- Gripper motor
- Lift motor
- Tracked drivetrain

## Software

Arduino / C++ using the Makeblock MegaPi library.
