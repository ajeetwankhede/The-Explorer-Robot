cmake_minimum_required(VERSION 2.8.3)
project(the_explorer_robot)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  sensor_msgs
  message_generation
)

catkin_package()

## Specify additional locations of header files
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(planner src/main.cpp src/map.cpp src/optimalPlanner.cpp src/output.cpp)
 add_executable(explorer src/explorer.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(planner
   ${catkin_LIBRARIES}
 )
  target_link_libraries(explorer
   ${catkin_LIBRARIES}
 )

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
    plannerTest
    test/plannerTest.launch
    test/main.cpp
    test/optimalPlannerTest.cpp
    test/mapTest.cpp
    test/outputTest.cpp
    src/map.cpp
    src/optimalPlanner.cpp
    src/output.cpp
  )

  target_link_libraries(
    plannerTest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    plannerTest
    planner
    ${catkin_EXPORTED_TARGETS}
  )
endif()
