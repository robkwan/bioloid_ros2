# bioloid_ros2
![Project Logo](path/to/logo.png)  
*ROSification of Robotis Premium Type A as the final project ot robotics class*

[![](https://github.com/robkwan/bioloid_ros2/)](https://github.com/robkwan/bioloid_ros2)  
[![](https://img.shields.io/badge/PRs-Welcome-brightgreen.svg)](https://github.com/robkwan/bioloid_ros2/pulls)  

---

## Table of Contents
1. [Features](#features)  
2. [Installation](#installation)  
3. [Usage](#usage)  

---

## Features
- **Feature 1**: All the codes are for ROS2 Jazzy / Ubuntu 24.04.  
- **Feature 2**: Running on Raspberry Pi 5 with ARM architecture but should be portable to x86_64 architecture for better performance.  

## Installtion
1. Clone the repo:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/robkwan/bioloid_ros2.git
   ```
   
2. Build the codes:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
   
3. Set up the ROS environment variables:
   ```bash
   source install/setup.bash
   ```

## Usage

### For Real Robotis Premium Type A Robot

Prerequisites:
- U2D2 communication module is connected to the USB port of the Raspberry Pi on one end
  while the other end is to use the 3-pin TTL connection to connect to the DXL port of the robot.
- The robot is power-up-ed already as U2D2 itself would not give power to the Dynamixel Servos.
  
1. In terminal 1,
   ```bash
   ros2 launch bioloid_description rviz.launch
   ```
   the rviz2 will then be launched.
   
2. In terminal 2.
   ```bash
   ros2 launch dynamixe_bringup dxl_bringup.launch.py
   ```
   the dynamixel ros2 control will then be launched.
   
3. In terminal 3,
   ```bash
   cd ~/ros2_ws/src/bioloid_ros2/bioloid_demos/real
   ./test_motion.py bow.mtn or l_forward_walk.lst
   ```
   the real robot will move according to motion selected and the .mtn/.lst are available in ~/bioloid_demos/motions folder
   
Notes;
- Unfortunately, sometimes not all the 18 motors can't be detected correctly
  but overall, it should still be functional ok.

   
### For Gazebo Sim (Harmonic) with cmd_pos topic

1. In terminal 1,
   ```bash
   ros2 launch bioloid_description rviz_gazebo.launch
   ```
   the rviz2 will then be launched.
   
2. In terminal 2.
   ```bash
   ros2 launch bioloid_gazebo gazebo_cmdpos.launch.py
   ```
   the gazebo sim will then be launched.
   
3. In terminal 3,
   ```bash
   cd ~/ros2_ws/src/bioloid_ros2/bioloid_demos/sim
   ./test_cmdpos_mtn.py bow.mtn or l_forward_walk.lst [--feedback]
   ```
   the simulated robot will move according to motion selected and the .mtn/.lst are available in ~/bioloid_demos/motions folder.
   --feedback option is to wait for each step of the motion to be closer to the goal position before executing the next step.
  
Notes:
  Two limitations here for the Gazebo simulation:
  i.  The motion execution is very slow and so --feedback option is added to safeguard the execution of a step before moving to the next step.
  ii. gravity is set to 0 and so the robot will float up.
      If it is set to gravity: -9.81 as usual, the robot will tumble easily.
      No easy tuning to find a gravity that would give balance so far.

      
### For Gazebo Sim (Harmonic) with gz sim ros2 control

1. In terminal 1,
   ```bash
   ros2 launch bioloid_description rviz_gazebo.launch
   ```
   the rviz2 will then be launched.
   
2. In terminal 2.
   ```bash
   ros2 launch bioloid_gazebo gazebo_ros2ctl.launch.py
   ```
   the gazebo sim will then be launched.
   
3. In terminal 3,
   ```bash
   cd ~/ros2_ws/src/bioloid_ros2/bioloid_demos/sim
   ./test_ros2ctl_mtn.py bow.mtn or l_forward_walk.lst [--feedback]
   ```
   the simulated robot will move according to motion selected and the .mtn/.lst are available in ~/bioloid_demos/motions folder.
   --feedback option is to wait for each step of the motion to be closer to the goal position before executing the next step.
  
Notes:
  Two limitations here for the Gazebo simulation:
  i.  The motion execution is very slow and so --feedback option is added to safeguard the execution of a step before moving to the next step.
  ii. gravity is set to -9.81 and so the robot will tumble easily.
      

### For Mujoco Simulator with mujoco_ros2_control

1. In terminal 1,
   ```bash
   ros2 launch bioloid_mujoco mujoco_ros2ctl.launch.py
   ```
   the mujoco sim will then be launched.
   
3. In terminal 2,
   ```bash
   cd ~/ros2_ws/src/bioloid_ros2/bioloid_demos/sim
   ./test_ros2ctl_mtn.py bow.mtn or l_forward_walk.lst [--feedback]
   ```
   the simulated robot will move according to motion selected and the .mtn/.lst are available in ~/bioloid_demos/motions folder.
   --feedback option is to wait for each step of the motion to be closer to the goal position before executing the next step.
  
Notes:
  Unlike the Gazebo sim, even though mujoco has the gravity set to -9.81, 
  the robot would not tumble as it goes through the motion steps even though it can still be very slow.
      
