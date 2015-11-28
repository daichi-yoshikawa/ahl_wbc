cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_controller)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_digital_filter
    ahl_robot
    ahl_utils
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

generate_dynamic_reconfigure_options(
  cfg/Param.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_robot_controller
  CATKIN_DEPENDS
    ahl_digital_filter
    ahl_robot
    ahl_utils
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
  DEPENDS
    Eigen
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

add_library(
  ahl_robot_controller
    src/robot_controller.cpp
    src/param.cpp
    src/common/effective_mass_matrix3d.cpp
    src/task/multi_task.cpp
    src/task/damping.cpp
    src/task/friction_compensation.cpp
    src/task/gravity_compensation.cpp
    src/task/joint_limit.cpp
    src/task/joint_control.cpp
    src/task/orientation_control.cpp
    src/task/position_control.cpp
)

add_dependencies(
  ahl_robot_controller
    ahl_robot_controller_gencfg
)

target_link_libraries(
  ahl_robot_controller
    ${catkin_LIBRARIES}
)
