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
 *  @file    map.hpp
 *  @author  Ajeet Wankhede (driver) and Likhita Madiraju (navigator)
 *  @date    11/27/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Final Project, The Explorer Robot
 *
 *  @section DESCRIPTION
 *
 *  This autonomous robotic system can be applied for TurtleBot in any indoor  
 *  facility for 2D mapping using SLAM and path planning.
 *  This is a Map class declaration file
 */

#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <map>
#include <vector>
#include <utility>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Map {
 public:
  /**
   *   @brief Default constructor for Map
   *
   *   @param nothing
   *   @return nothing
   */
  Map();
  /**
   *   @brief Default destructor for Map
   *
   *   @param nothing
   *   @return nothing
   */
  ~Map();
  /**
   *   @brief This is a callback function for laser data
   *
   *   @param LaserScan message
   *
   *   @return none
   */
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   *   @brief Use this function to create the map
   *
   *   @param none
   *
   *   @return none
   */
  void createMap();
  bool obstacle;
  std::pair<int, int> startNode;
  std::pair<int, int> endNode;

 private:
  geometry_msgs::Twist msg;
  ros::NodeHandle n;
  ros::Publisher velocity_pub;
  ros::Subscriber sub;
  float speedX, rotateZ;
};

#endif  // INCLUDE_MAP_HPP_
