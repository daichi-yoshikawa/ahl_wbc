cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_samples)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
    visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Pr2Param.cfg
  cfg/YouBotParam.cfg
)

catkin_package(
  CATKIN_DEPENDS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(
  pr2_sample
    src/pr2/main.cpp
    src/pr2/pr2.cpp
    src/marker/marker.cpp
    src/marker/markers.cpp
)

add_dependencies(
  pr2_sample
    gazebo_msgs_gencpp
    ahl_robot_samples_gencfg
)

target_link_libraries(
  pr2_sample
    ${catkin_LIBRARIES}
)

#add_executable(
#  pr2_hybrid_sample
#    src/pr2/main_hybrid.cpp
#    src/pr2/pr2_hybrid.cpp
#    src/marker/marker.cpp
#    src/marker/markers.cpp
#)

#add_dependencies(
#  pr2_hybrid_sample
#    gazebo_msgs_gencpp
#    ahl_robot_samples_gencfg
#)

#target_link_libraries(
#  pr2_hybrid_sample
#    ${catkin_LIBRARIES}
#)

add_executable(
  youbot_sample
    src/youbot/main.cpp
    src/youbot/youbot.cpp
    src/marker/marker.cpp
    src/marker/markers.cpp
)

add_dependencies(
  youbot_sample
    gazebo_msgs_gencpp
    ahl_robot_samples_gencfg
)

target_link_libraries(
  youbot_sample
    ${catkin_LIBRARIES}
)
