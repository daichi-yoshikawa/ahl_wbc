cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_digital_filter
    ahl_utils
    roscpp
    tf2
    tf2_ros
)

find_package(
  cmake_modules REQUIRED
)

find_package(PkgConfig)

pkg_check_modules(NEW_YAMLcpp yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

find_package(
  Eigen REQUIRED
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_robot
  CATKIN_DEPENDS
    ahl_digital_filter
    ahl_utils
    roscpp
    tf2
    tf2_ros
  DEPENDS
    Eigen
    yaml-cpp
)

add_library(
  ahl_robot
    src/robot/robot.cpp
    src/robot/manipulator.cpp
    src/robot/transformation.cpp
    src/robot/parser.cpp
    src/tf/tf_publisher.cpp
    src/utils/math.cpp
)

target_link_libraries(
  ahl_robot
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(
  parser_test
    test/test.cpp
)

add_dependencies(
  parser_test
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test
    ahl_robot
)

add_executable(
  parser_test2
    test/test2.cpp
)

add_dependencies(
  parser_test2
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test2
    ahl_robot
)

add_executable(
  parser_test3
    test/test3.cpp
)

add_dependencies(
  parser_test3
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test3
    ahl_robot
)

add_executable(
  parser_test4
    test/test4.cpp
)

add_dependencies(
  parser_test4
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test4
    ahl_robot
)

add_executable(
  parser_test5
    test/test5.cpp
)

add_dependencies(
  parser_test5
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test5
    ahl_robot
)

add_executable(
  parser_test6
    test/test6.cpp
)

add_dependencies(
  parser_test6
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test6
    ahl_robot
)
