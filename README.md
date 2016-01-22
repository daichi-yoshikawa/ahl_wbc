ahl_wbc
========
ahl_wbc is a metapackage that contains ROS packages
for whole body control/operational space control/task space control.

What is whole body control ?
=========================================================================
If you use this package,
* you don't need inverse kinematics
* you can easily control robots with many joints
* you can add force control with the same framework

Reference : [The operational space framework](http://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1993_JSME.pdf)

Prerequisites
=============
sudo apt-get install ros-<ros-version>-ros-control ros-<ros-version>-ros-controllers

eg.)
sudo apt-get install ros-jade-ros-control ros-jade-ros-controllers

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



