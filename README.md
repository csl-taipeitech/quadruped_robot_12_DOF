# How to Run the Files

## Step 1: Launch the Gait Generator

Open the `champ bringup` to activate the gait generator:

```bash
cd champ/
source install/setup.bash
ros2 launch fooldog_config bringup.launch.py
```

## Step 2: Run the Robot Program

Navigate to the robot's program directory and execute the control script:

```bash
cd champ/src/quadruped_robot_12_DOF/triceratops_base/
python3 Triceratops_ControlCmd.py
```

## Step 3: Control the Robot

You can control the robot using either a keyboard or a joystick.

### Keyboard Controller

Launch the keyboard controller:

```bash
cd champ/
source install/setup.bash
ros2 launch champ_teleop teleop.launch.py
```

### Joystick Controller

Run the joystick controller script:

```bash
cd champ/src/champ_teleop/
python3 joy_controller_new.py
```

Start the `joy_node` topic:

```bash
ros2 run joy joy_node
```
