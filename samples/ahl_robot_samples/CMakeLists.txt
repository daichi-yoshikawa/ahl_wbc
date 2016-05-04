cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_samples)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0x)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    cmake_modules
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
    visualization_msgs
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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
    cmake_modules
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
    visualization_msgs
)

add_executable(pr2_sample
  src/pr2/main.cpp
  src/pr2/pr2.cpp
  src/marker/marker.cpp
  src/marker/markers.cpp
)
add_dependencies(pr2_sample
  gazebo_msgs_gencpp
  ahl_robot_samples_gencfg
)
target_link_libraries(pr2_sample ${catkin_LIBRARIES})

add_executable(youbot_sample
  src/youbot/main.cpp
  src/youbot/youbot.cpp
  src/marker/marker.cpp
  src/marker/markers.cpp
  src/mobility/mecanum_wheel.cpp
)
add_dependencies(youbot_sample
  gazebo_msgs_gencpp
  ahl_robot_samples_gencfg
)
target_link_libraries(youbot_sample ${catkin_LIBRARIES})
