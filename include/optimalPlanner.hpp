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
 *  @file    optimalPlanner.hpp
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
 *  This is a OptimalPlanner class declaration file
 */

#ifndef INCLUDE_OPTIMALPLANNER_HPP_
#define INCLUDE_OPTIMALPLANNER_HPP_

#include <map>
#include <vector>
#include <utility>

/**
 * @brief Declaring class attributes and methods
 */
class OptimalPlanner {
 public:
  /**
   * @brief Constructor
   */
  OptimalPlanner();

  /**
   * @brief Destructor
   */
  virtual ~OptimalPlanner();

  /**
   *   @brief Call this method to initialize different variables regarding to the world
   *   @param none
   *   @return none
   */
  void initMap();

  /**
   *   @brief Run the search method to find a path from robot pose to goal pose
   *   @param vector of vector of int containin the world state
   *   @param pair of int of the robot pose or start node
   *   @param pair of int of the goal pose or end node
   *   @return vector of pair of int containing the path found by the search algorithm.
   */
  std::vector<std::pair<int, int> > search(
      std::vector<std::vector<int> > worldState, std::pair<int, int> robotPose,
      std::pair<int, int> goalPose);
  /**
   *   @brief Verify that the given node is not visited
   *   @param pair of int of the node to be checked
   *   @return bool value specifying valid or invalid nodes
   */
  bool verifyNodes(std::pair<int, int> node);

  /**
   *   @brief Decide which action to take in the A* algorithm
   *   @param int value to decide which action to take
   *   @param pair of int value of current node
   *   @return pair of int value specifying new node
   */
  std::pair<int, int> action(int i, std::pair<int, int> currentNode);

  std::vector<std::pair<int, int> > path;
  std::map<std::pair<int, int>, int> visitedNodes;
  std::map<std::pair<int, int>, double> parentCost;
  std::map<std::pair<int, int>, double> currentCost;
  std::map<std::pair<int, int>, std::pair<int, int> > parentNode;
  int length, width;
  std::vector<std::vector<int> > world;
 private:
  int noOfNodes;
};

#endif  // INCLUDE_OPTIMALPLANNER_HPP_
