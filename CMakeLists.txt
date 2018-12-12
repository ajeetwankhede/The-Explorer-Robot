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

## Generate services in the 'srv' folder
add_service_files(FILES change_speed.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

## Specify additional locations of header files
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(explorer src/main.cpp src/explorer.cpp)

## Specify libraries to link a library or executable target against
  target_link_libraries(explorer
   ${catkin_LIBRARIES}
 )

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
    explorerTest
    test/explorerTest.launch
    test/main.cpp
    test/explorerTest.cpp
    src/explorer.cpp
  )

  target_link_libraries(
    explorerTest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    explorerTest
    explorer
    the_explorer_robot_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )
endif()
