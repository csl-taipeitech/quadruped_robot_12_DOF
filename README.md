Bionic Quadruped Robot Control System
Project Objective
This project aims to achieve motion control for a bionic quadruped robot through the design of a Joystick Interface, Controller, and Motor Control.

Project Overview


🔹 Video of the quadruped robot in action:

How to Run the Files
Step 1: Run the Robot Program
Open a terminal.

Navigate to the robot's program directory and execute the control script:

bash
Copy
Edit
cd quadruped_robot_12_DOF/triceratops_quadruped_robot/triceratops_base/
python3 Controller.py
Step 2: Control the Robot
You can control the robot using a joystick.

Joystick Controller
Open another terminal.

Run the joystick controller script:

bash
Copy
Edit
cd champ/src/champ_teleop/
python3 Joy_controller.py
Open another terminal.

Start the joy_node topic:

bash
Copy
Edit
ros2 run joy joy_node
System Architecture and Workflow


The main program, Controller.py, runs in a loop and controls the quadruped robot’s movements through the Joystick Interface, Controller, and Hardware Interface.

1. Joystick Interface
Receives joystick signals via Bluetooth and outputs the current state through ROS2 topics.
Controls the robot’s forward, backward, left, and right movements.
2. Controller
Gait Generator: Generates the robot’s gait trajectory. Currently supports Trot gait.
Inverse Kinematics: Computes joint angles corresponding to the gait trajectory and calculates motor output angles using a linkage model.
3. Hardware Interface
Uses Dynamixel Motors to transmit calculated angles to the motors, enabling the robot to walk.
Gait Control
The gait control system consists of four core modules:

1. Gait Scheduler
Determines the state of each leg (stance or swing phase).
In Trot gait, diagonal legs move synchronously and alternate between stance and swing phases.
2. Stance Phase Controller
Controls the legs in contact with the ground, adjusting movement based on the robot’s target velocity.
Ensures support legs move in the opposite direction of the robot’s forward motion.
Adjusts support leg rotation for stable turning.
3. Swing Phase Controller
Lifts the legs that just completed the stance phase and moves them to the next landing position.
Selects landing positions based on the equal displacement principle between swing and stance phases, ensuring stable gait motion.
4. Inverse Kinematics Model
Calculates the target position of legs in Cartesian coordinates and converts them into corresponding joint angles.
These motor angles are sent to the robot’s state variables to drive movement.
Motor Control
Motor Type: Dynamixel Motors
Baud Rate Setting: 3M
PID Parameter Settings:
P gain: 8000
I gain: 0
D gain: 0
Future Prospects
✅ Integrating IMU to improve the robot’s balance.
✅ Adding posture variations and more gait patterns.
✅ Optimizing gait tracking using alternative control methods.
✅ Enhancing perception capabilities for better environmental adaptability.

References
1. Open-Source Projects
Stanford Pupper: Official Website
2. Chinese Learning Materials
[Tutorial] DengGe's Beginner DIY Quadruped Robot Guide (New Version)
📺 Bilibili Tutorial Video
Gait
Chapter 3.1: Trot Gait
Chapter 3.8: Trot Gait Program Implementation
Inverse Kinematics
Chapter 3.5: 12-DOF Kinematic Inverse Solution
3. English Learning Materials
Gait Control

Legged Robotics Lec14a: Raibert foot placement control for hopper (Spring 2021)
📺 YouTube Link
Inverse Kinematics

Inverse Kinematics for SpotMicro Robotics | Example and Demo
📺 YouTube Link
