ahl_wbc
========
ahl_wbc is a metapackage that contains ROS packages
for whole body control/operational space control/task space control.

What is Whole Body Control ?
============================
If you use this package,
* you can easily control robots with many joints (See video clips below!)
* you can add force control with the same framework
* you don't need inverse kinematics

Reference : [The operational space framework](http://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1993_JSME.pdf)

Contents
=========
### Packages
* wbc/ahl_robot :
  Implement kinematics including transformation, computation of Jacobian and mass matrix.
* wbc/ahl_robot_controller : Provide joint space control and task space control
* samples/ahl_youbot_description : Contain urdf description of YouBot
* samples/ahl_pr2_description : Contain urdf description of PR2
* samples/ahl_red_arm : Contain sdf description of original red arm

How does it work ?
===================
* Task space control of PR2.                   
[![](http://img.youtube.com/vi/7pHPHKFTwZs/0.jpg)](https://www.youtube.com/watch?v=7pHPHKFTwZs)

* Task space control of mobile manipulator(Kuka YouBot).   
[![](http://img.youtube.com/vi/RHdLje50RXQ/0.jpg)](https://www.youtube.com/watch?v=RHdLje50RXQ)

* Task space control of 7 DOF manipulator.   
[![](http://img.youtube.com/vi/v_i-LgaJ5WM/0.jpg)](https://www.youtube.com/watch?v=v_i-LgaJ5WM)

* Task space control of 11 DOF manipulator.   
[![](http://img.youtube.com/vi/oKqCsFAzx4k/0.jpg)](https://www.youtube.com/watch?v=oKqCsFAzx4k)

Prerequisites
=============
* [Ubuntu 14.04 LTS](http://www.ubuntu.com/download)
* [ROS Indigo](http://wiki.ros.org/), Jade (Jade is preferable)
* [Gazebo 5.0](http://gazebosim.org/) or later
* [ros-control](http://wiki.ros.org/ros_control) and [ros-controllers](http://wiki.ros.org/ros_controllers) (for gazebo simulation)

Installation
=============
### Install Ubuntu 14.04 LTS
[Official documentation](http://www.ubuntu.com/download/desktop/install-ubuntu-desktop)

### Install ROS Jade
ROS is an open source middle ware for robot developers.<br>
It serves vast amount of useful features and packages.<br>
If you have enough strage, install ros-jade-desktop-full.<br>
[Official documentation](http://wiki.ros.org/jade/Installation/Ubuntu)

### Create Catkin Workspace
Catkin is a cmake-based build system supported by ROS.<br>
In catkin workspace, we edit codes and build software.<br>
[Official documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Install Gazebo 5.0
Gazebo is a robotic simulator, which provides visualization,
physics and collision detection in virtual environment.
```
sudo apt-get install gazebo5
```

### Install ros-control and ros-controllers
In order to fully make use of gazebo with ROS, we need these controllers.
```
sudo apt-get install ros-jade-ros-control ros-jade-ros-controllers
```

### Install ROS Packages for Whole Body Control.
In your catkin_ws/src/ahl_ros_packages, execute the follows.
```
git clone https://github.com/daichi-yoshikawa/existing_packages.git
git clone https://github.com/daichi-yoshikawa/ahl_common.git
git clone https://github.com/daichi-yoshikawa/ahl_wbc.git
```

Get Started with Gazebo Simulation
==================================
You need two terminals.

Firstly, you have to launch ros node which provides virtual robot model.<br>
In this phase, no window will pop up, because gazebo GUI is off by default.<br>

If you'd like to simulate KUKA youBot,
```
roslaunch ahl_youbot_description youbot.launch
```
Or if you'd like to simulate PR2,
```
roslaunch ahl_pr2_description pr2.launch
```

And then, launch the whole body controller.<br>
If you make success, rviz will pop up and you can see robot model in it.<br>
If you launched youbot.launch at the previous step,
```
roslaunch ahl_robot_samples youbot.launch
```
Or if you launched pr2.launch at the previous step,
```
roslaunch ahl_robot_samples pr2.launch
```

Note : You have to tune parameters in order to make it fine. See the following section "Tuning parameters".

Tuning parameters
=================
Actually, default control gains are very low, so the robot is not properly controlled.
It is because simulator would get unstable, if gains were set to high values from the beginning.

Therefore, you need to increase gains after you launch simulator.
It can be done with window named "rqt_reconfigure__Param - rqt".

Case youbot simulation : Select youbot_sample->ahl_robot_controller in the window and set parameters as the follows.
```
kp_joint = 50.0
kv_joint = 5.0
kp_task_pos = 70.0
ki_task_pos = 0.0
kv_task_pos = 5.0
kp_task_ori = 200.0
ki_task_ori = 0.0
kv_task_ori = 5.0 
```

Usage
=====
ahl_robot_samples is a good reference to know how to use APIs.<br>
You have two ways to control physical robots.

1. Based on input of desired torques at joints (Torque sensors and solid torque control is MUST.)
2. Based on input of desired angles at joints (Works without torque sensors but cannot control a force)

ahl_ctrl::RobotController::computeGeneralizedForce derives desired torques.<br>
ahl_ctrl::RobotController::simulate computes desired joint angles based on torques derived by ahl_ctrl::RobotController::computeGeneralizedForce.

Most robots don't have torque sensor on it. Therefore ahl_ctrl::RobotController::simulate will help you.

### Gazebo Simulation
ahl_youbot_description and ahl_pr2_description are good references to setup robot model for simulation.<br>

### Physical Robot (***Editing***)
Be careful ! Torque control is pretty difficult !!
