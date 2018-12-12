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
 *  @file    explorer.hpp
 *  @author  Ajeet Wankhede
 *  @date    12/03/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Week 12, Working with Gazebo
 *
 *  @section DESCRIPTION
 *
 *  This autonomous robotic system can be applied for TurtleBot in any indoor  
 *  facility for 2D mapping using SLAM and path planning.
 *  This is a explorer class header file
 */

#ifndef INCLUDE_EXPLORER_HPP_
#define INCLUDE_EXPLORER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "the_explorer_robot/change_speed.h"

class Explorer {
 public:
  /**
   *   @brief Default constructor for Explorer
   *
   *   @param nothing
   *   @return nothing
   */
  Explorer();
  /**
   *   @brief Default destructor for Explorer
   *
   *   @param nothing
   *   @return nothing
   */
  ~Explorer();
  /**
   *   @brief This is a callback function for laser data
   *
   *   @param LaserScan message
   *
   *   @return none
   */
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
 *   @brief Service for changing the speed
 *
 *   @param req: request by client
 *   @param res: request of the server
 *
 *   @return bool
 */
  bool changeSpeed(the_explorer_robot::change_speed::Request& req,
                   the_explorer_robot::change_speed::Response& res);
  /**
   *   @brief Use this function to start the robot
   *
   *   @param none
   *
   *   @return none
   */
  void explore();
  bool obstacle;

 private:
  geometry_msgs::Twist msg;
  ros::NodeHandle n;
  ros::Publisher velocity_pub;
  ros::Subscriber sub;
  ros::ServiceServer server;
  float speedX, rotateZ;
  float message;
};

#endif  // INCLUDE_EXPLORER_HPP_
