cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot)

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
    ahl_digital_filter
    ahl_utils
    cmake_modules
    roscpp
    tf2
    tf2_ros
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLcpp yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

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
  INCLUDE_DIRS include
  LIBRARIES ahl_robot
  CATKIN_DEPENDS
    ahl_digital_filter
    ahl_utils
    cmake_modules
    roscpp
    tf2
    tf2_ros
  DEPENDS yaml-cpp
)

add_library(ahl_robot
  src/robot/robot.cpp
  src/robot/manipulator.cpp
  src/robot/transformation.cpp
  src/robot/parser.cpp
  src/robot/tf_publisher.cpp
  src/utils/math.cpp
)

target_link_libraries(ahl_robot
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(parser_test test/test.cpp)
add_dependencies(parser_test ahl_robot_gencpp)
target_link_libraries(parser_test ahl_robot)

add_executable(parser_test2 test/test2.cpp)
add_dependencies(parser_test2 ahl_robot_gencpp)
target_link_libraries(parser_test2 ahl_robot)

add_executable(parser_test3 test/test3.cpp)
add_dependencies(parser_test3 ahl_robot_gencpp)
target_link_libraries(parser_test3 ahl_robot)

add_executable(parser_test4 test/test4.cpp)
add_dependencies(parser_test4 ahl_robot_gencpp)
target_link_libraries(parser_test4 ahl_robot)

add_executable(parser_test5 test/test5.cpp)
add_dependencies(parser_test5 ahl_robot_gencpp)
target_link_libraries(parser_test5 ahl_robot)

add_executable(parser_test6 test/test6.cpp)
add_dependencies(parser_test6 ahl_robot_gencpp)
target_link_libraries(parser_test6 ahl_robot)
