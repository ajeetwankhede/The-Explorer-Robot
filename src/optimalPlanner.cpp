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
 *  @file    optimalPlanner.cpp
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
 *  This is a OptimalPlanner class implementation file
 */

#include <iostream>
#include <vector>
#include <utility>
#include <algorithm>
#include "optimalPlanner.hpp"

using std::vector;
using std::pair;
using std::make_pair;

OptimalPlanner::OptimalPlanner() {
  // Initializing values to the attributes of OptimalPlanner class
  noOfNodes = 0;
  length = 10;
  width = 10;
}

OptimalPlanner::~OptimalPlanner() {
  // Destructor stub
}

void OptimalPlanner::initMap() {
  // Create new key value pairs
  // Create new key value pair and initialize to zero if key not found
}

vector<pair<int, int> > OptimalPlanner::search(vector<vector<int> > worldState,
                                              pair<int, int> robotPose,
                                              pair<int, int> goalPose) {
  // Assign the vales to OptimalPlanner's attribute
  // Call the initMap to initialize the variables
  // Verify the start and goal pose to be within map boundaries
  // and outside obstacle space
  // Start the search
  return path;
}

bool OptimalPlanner::verifyNodes(pair<int, int> node) {
  bool verify = false;
  // Verify if new node is inside the boundaries of map
  // Verify if new node is not inside the
  // obstacle space or visited before
  return verify;
}

pair<int, int> OptimalPlanner::action(int i, pair<int, int> currentNode) {
  return make_pair(0, 0);
}
