# Quadruped Robot 12 DOF

It can walk with linear and angular velocity, and it also allows body pose input for pose adjustment, as well as a handshake pose for human interaction.

![Untitled ‑ Made with FlexClip (68)](https://github.com/user-attachments/assets/21a16aaa-f6ab-4b26-9ec4-13107c00cee7)    ![Untitled ‑ Made with FlexClip (71)](https://github.com/user-attachments/assets/75dfde75-53cd-493b-81c7-7e43cde3c13b)


## How to Run the Files

### Step 1: Launch the Gait Generator

1. Open a terminal
2. Open the `champ bringup` to activate the gait generator:

```bash
cd champ/
source install/setup.bash
ros2 launch fooldog_config bringup.launch.py
```

### Step 2: Run the Robot Program
Body_pose & hand-shake pose

NOTICE:MAKE SURE YOU ARE NOT IN CONDA ENVIRONMENT!!
if you see (base)
```
cd ~
cd champ/src/champ_teleop/
python3 stanley_joy_stick.py
```

1. Open another terminal
2. Navigate to the robot's program directory and execute the control script:

```bash
cd ~
cd champ/src/champ_teleop/
python3 Triceratops_ControlCmd.py
```
press CMD: s while python3 Triceratops_ControlCmd.py to enable motor

### Step 3: Control the Robot

You can control the robot using joystick.

#### Joystick Controller
![joystick](https://github.com/user-attachments/assets/8dd15f47-b1a1-47a8-9a94-0dbaa0c32bd4)

1. Open another terminal
2. Run the joystick controller script:
Start the `joy_node` topic:

```
ros2 run joy joy_node
```

#### Gait Adjustment
![Untitled ‑ Made with FlexClip (72)](https://github.com/user-attachments/assets/f578d0f1-98fe-493c-8229-db06f9273556)

Phase Generator
```
cd champ/champ/include/champ/leg_controller/phase_generator.h
```

Trot/Crawl/Gallop
```
//Trot
leg_clocks[0] = elapsed_time_ref - (0.0f * stride_period);
leg_clocks[1] = elapsed_time_ref - (0.5f * stride_period);
leg_clocks[2] = elapsed_time_ref - (0.5f * stride_period);
leg_clocks[3] = elapsed_time_ref - (0.0f * stride_period);

//Crawl
// leg_clocks[0] = elapsed_time_ref - (0.0f * stride_period);  // Front-Right
// leg_clocks[1] = elapsed_time_ref - (0.25f * stride_period); // Front-Left
// leg_clocks[2] = elapsed_time_ref - (0.50f * stride_period); // Back-Right
// leg_clocks[3] = elapsed_time_ref - (0.75f * stride_period); // Back-Left

//Gallop
// leg_clocks[0] = elapsed_time_ref - (0.5f * stride_period);  // Front-Right
// leg_clocks[1] = elapsed_time_ref - (0.5f * stride_period);  // Front-Left
// leg_clocks[2] = elapsed_time_ref - (0.0f * stride_period);  // Back-Right
// leg_clocks[3] = elapsed_time_ref - (0.0f * stride_period);  // Back-Left
```

Gait.yaml
```
cd champ/fooldog_config/config/gait/gait.yaml
```

```
##Trot
/**:
  ros__parameters:
    gait:
      knee_orientation : "><"
      pantograph_leg : false
      odom_scaler: 1.0 
      max_linear_velocity_x : 0.15 #0.1
      max_linear_velocity_y : 0.065 #0.05
      max_angular_velocity_z : 1.0 #0.1
      com_x_translation : 0.0 #0.001
      swing_height : 0.02 #0.04
      stance_depth : 0.0 #0.0
      stance_duration : 0.55 #0.5
      nominal_height : 0.16 #0.16

##Crawl
# /**:
#   ros__parameters:
#     gait:
#       knee_orientation : "><"
#       pantograph_leg : false
#       odom_scaler: 1.0 
#       max_linear_velocity_x : 0.1  # Crawl is much slower
#       max_linear_velocity_y : 0.06
#       max_angular_velocity_z : 0.6  # Slow turning

#       com_x_translation : 0.0
#       swing_height : 0.015  # 0.01
#       stance_depth : 0.0

#       stance_duration : 0.85  # 0.9
#       nominal_height : 0.145 #0.16

#       phase_offsets: [0.0, 0.25, 0.50, 0.75]  # Change from Trot to Crawl

##Gallop
# /**:
#   ros__parameters:
#     gait:
#       knee_orientation : "><"
#       pantograph_leg : false
#       odom_scaler: 1.5  # Faster speed
#       max_linear_velocity_x : 0.7  # Increase speed for gallop
#       max_linear_velocity_y : 0.2
#       max_angular_velocity_z : 1.2  

#       com_x_translation : 0.0
#       swing_height : 0.05  # Higher lift for gallop
#       stance_depth : 0.0

#       stance_duration : 0.35  # Short stance, fast movement
#       nominal_height : 0.15  

#       phase_offsets: [0.0, 0.0, 0.5, 0.5]  # Hind legs together, front legs together
```
