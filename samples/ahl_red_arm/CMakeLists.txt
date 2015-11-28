cmake_minimum_required(VERSION 2.8.3)
project(ahl_red_arm)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ahl_red_arm
#  CATKIN_DEPENDS ahl_gazebo_interface ahl_robot ahl_robot_controller roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_library(ahl_red_arm
#   src/${PROJECT_NAME}/ahl_red_arm.cpp
# )

# add_executable(ahl_red_arm_node src/ahl_red_arm_node.cpp)

# add_dependencies(ahl_red_arm_node ahl_red_arm_generate_messages_cpp)

# target_link_libraries(ahl_red_arm_node
#   ${catkin_LIBRARIES}
# )
