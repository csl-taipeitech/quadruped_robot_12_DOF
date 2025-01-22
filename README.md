# How to Run the Files

## Step 1: Run the Robot Program

1. Open a terminal
2. Navigate to the robot's program directory and execute the control script:

```bash
cd quadruped_robot_12_DOF/triceratops_quadruped_robot/triceratops_base/
python3 Controller.py
```

## Step 2: Control the Robot

You can control the robot using a joystick.

### Joystick Controller

1. Open another terminal
2. Run the joystick controller script:

```bash
cd champ/src/champ_teleop/
python3 Joy_controller.py
```

1. Open another terminal
2. Start the `joy_node` topic:

```bash
ros2 run joy joy_node
```
