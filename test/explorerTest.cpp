/************************************************************************
 MIT License
 Copyright (c) 2018 Ajeet Wankhede
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    explorerTest.cpp
 *  @author  Ajeet Wankhede (driver) and Likhita Madiraju (navigator)
 *  @date    12/10/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Final Project, The Explorer Robot
 *
 *  @section DESCRIPTION
 *
 *  This is a test source file for testing Explorer class
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_srvs/Empty.h>
#include <ros/service_client.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "the_explorer_robot/change_speed.h"
#include "explorer.hpp"

/**
 * @brief This is a test to check the existence of ROS service change_speed 
 * provided by explorer node
 */
TEST(testexplorerTest, serviceExistenceTest) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Setup a client to the service
  ros::ServiceClient client =
      n.serviceClient<the_explorer_robot::change_speed>("change_speed");
  // Check if the service created by talker node exists
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/**
 * @brief This is a test to check if the change speed service works correctly
 */
TEST(testexplorerTest, serviceTest) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Setup a client to request the service
  ros::ServiceClient client =
      n.serviceClient<the_explorer_robot::change_speed>("change_speed");
  // Create a service object
  the_explorer_robot::change_speed srv;
  // Assign the speed that needs to be changed
  srv.request.newSpeedX = 2.0;
  srv.request.newRotateZ = 1.0;
  // Call the service
  client.call(srv);
  // Check the response
  EXPECT_EQ(2.0, srv.response.respSpeedX);
  EXPECT_EQ(1.0, srv.response.respRotateZ);
}

/**
 * @brief This is a test to check if the obstacle is correctly initialized
 */
TEST(explorerTest, initTest) {
  Explorer testObject;
  EXPECT_FALSE(testObject.obstacle);
  EXPECT_TRUE(testObject.turn);
  EXPECT_EQ(0, testObject.count);
}

/**
 * @brief This is a test to check if the sensorCallback works correctly
 */
//TEST(testexplorerTest, sensorCallbackTest) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  /* ros::NodeHandle n;
   ros::Subscriber sub;
   Explorer testObject;
   testObject.sensorCallback(sensor_msgs::LaserScan::ConstPtr& msg);
   // Check the obstacle status
   EXPECT_TRUE(testObject.obstacle);
}*/

/**
 * @brief This is a test to check if the planner works correctly
 */
TEST(testexplorerTest, plannerTest1) {
  Explorer testObject;
  testObject.obstacle = true;
  testObject.planner();
  // Check the msg
  EXPECT_EQ(testObject.msg.linear.x, 0.0);
  EXPECT_EQ(testObject.msg.angular.z, testObject.rotateZ);
}

/**
 * @brief This is a test to check if the planner works correctly
 */
TEST(testexplorerTest, plannerTest2) {
  Explorer testObject;
  testObject.planner();
  // Check the msg
  EXPECT_EQ(testObject.msg.linear.x, 0.0);
  EXPECT_EQ(testObject.msg.angular.z, testObject.rotateZ * 4);
  EXPECT_EQ(testObject.count, 1);
  testObject.count = 300;
  testObject.planner();
  EXPECT_EQ(testObject.msg.linear.x, testObject.speedX);
  EXPECT_EQ(testObject.msg.angular.z, 0.0);
  EXPECT_FALSE(testObject.turn);
  testObject.count = 700;
  testObject.planner();
  EXPECT_EQ(testObject.count, 1);
  testObject.planner();
  EXPECT_EQ(testObject.msg.linear.x, 0.0);
  EXPECT_EQ(testObject.msg.angular.z, -testObject.rotateZ * 4);  
}
