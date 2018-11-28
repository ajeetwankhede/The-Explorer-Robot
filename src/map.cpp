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
 *  @file    map.cpp
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
 *  This is a Map class implementation file
 */

#include <iostream>
#include <map>
#include <vector>
#include <utility>
#include "map.hpp"

Map::Map() {
  // Initializing values to the attributes of Map class
  obstacle = false;
  speedX = 0.1;
  rotateZ = 1.0;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
}

Map::~Map() {
  // Destructor stub
}

void Map::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Checking if the TurtleBot is close to obstacle
}

void Map::createMap() {
  // Move the TurtleBot avoiding obstacles to create a map
}
