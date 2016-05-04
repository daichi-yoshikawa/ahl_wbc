cmake_minimum_required(VERSION 2.8.3)
project(ahl_lwr_server)

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
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2
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

catkin_package(
  CATKIN_DEPENDS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    cmake_modules
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2
  DEPENDS
    Eigen3
)

add_executable(lwr_server_test test/test.cpp)
target_link_libraries(lwr_server_test ${catkin_LIBRARIES})

add_executable(red_arm_server_test test/test2.cpp)
target_link_libraries(red_arm_server_test ${catkin_LIBRARIES})

add_executable(red_arm2_server_test test/test3.cpp)
target_link_libraries(red_arm2_server_test ${catkin_LIBRARIES})
