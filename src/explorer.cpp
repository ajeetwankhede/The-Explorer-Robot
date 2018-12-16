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
 *  @file    explorer.cpp
 *  @author  Ajeet Wankhede (driver) and Likhita Madiraju (navigator)
 *  @date    12/03/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Final Project, The Explorer Robot
 *
 *  @section DESCRIPTION
 *
 *  This autonomous robotic system can be applied for TurtleBot in any indoor  
 *  facility for 2D mapping using SLAM and path planning.
 *  This is a Explorer class source file
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "the_explorer_robot/change_speed.h"
#include <explorer.hpp>

Explorer::Explorer() {
  // Initializing values to the attributes of Explorer class
  obstacle = false;
  speedX = 0.1;
  rotateZ = 0.1;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  message = 0.0;
  count = 0;
  turn = true;
}

Explorer::~Explorer() {
  // Destructor stub
}

void Explorer::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Checking if the TurtleBot is close to obstacle
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] <= 1.0) {
      obstacle = true;
      return;
    }
  }
  obstacle = false;
}

bool Explorer::changeSpeed(the_explorer_robot::change_speed::Request& req,
                  the_explorer_robot::change_speed::Response& res) {
  speedX = req.newSpeedX;
  rotateZ = req.newRotateZ;
  res.respSpeedX = speedX;
  res.respRotateZ = rotateZ;
  return true;
}

void Explorer::planner() {
  // Collision Detection check
  if (obstacle == true) {
    // Collision detected
    msg.linear.x = 0.0;
    msg.angular.z = rotateZ;
    ROS_INFO("Collision detected: ");
  } else {
    // No collision
    if (count < 300) {
      msg.linear.x = 0.0;
      if (turn) {
        msg.angular.z = rotateZ * 4;
      } else {
        msg.angular.z = -rotateZ * 4;
      }
    } else if (count >= 300 && count < 700) {
        turn = false;
        msg.angular.z = 0.0;
        msg.linear.x = speedX;
    } else {
      count = 0;
    }
    count++;
    ROS_INFO("No collision: ");
  }
}

void Explorer::explore() {
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  velocity_pub = n.advertise<geometry_msgs::Twist>
                                ("/cmd_vel_mux/input/navi", 1000);
  // Register our service with the master
  server = n.advertiseService("change_speed", &Explorer::changeSpeed, this);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  sub = n.subscribe <sensor_msgs::LaserScan>
       ("/scan", 300, &Explorer::sensorCallback, this);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    planner();
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    velocity_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
