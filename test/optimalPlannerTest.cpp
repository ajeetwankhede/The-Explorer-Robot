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
 *  @file    optimalPlannerTest.cpp
 *  @author  Ajeet Wankhede (driver) and Likhita Madiraju (navigator)
 *  @date    11/27/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Final Project, The Explorer Robot
 *
 *  @section DESCRIPTION
 *
 *  This is a test source file for testing optimalPlanner class
 */

#include <gtest/gtest.h>
#include <memory>
#include <map>
#include <vector>
#include <utility>
#include "optimalPlanner.hpp"

using std::pair;
using std::make_pair;
using std::vector;

/**
 * @brief Check if initMap method works
 */
TEST(OptimalPlanner, initMapTest) {
  OptimalPlanner OptimalPlannerTestObject;
  // Creating a map with no obstacles
  vector<vector<int> > worldState(4, vector<int>(4, 0));
  OptimalPlannerTestObject.world = worldState;
  OptimalPlannerTestObject.length = worldState[0].size();
  OptimalPlannerTestObject.width = worldState.size();
  // Call the initMap method
  OptimalPlannerTestObject.initMap();
  // Check if visitedNodes map is initialized
  EXPECT_FALSE(OptimalPlannerTestObject.visitedNodes.empty());
  // Check if parentCost map is initialized
  EXPECT_FALSE(OptimalPlannerTestObject.parentCost.empty());
  // Check if currentCost map is initialized
  EXPECT_FALSE(OptimalPlannerTestObject.currentCost.empty());
}

/**
 * @brief Check if search method works for a sample map
 */
TEST(OptimalPlanner, searchTest) {
  OptimalPlanner OptimalPlannerTestObject;
  // Creating a map with no obstacles
  vector<vector<int> > worldState(4, vector<int>(4, 0));
  // Set start and goal pose
  pair<int, int> robotPose = make_pair(1, 1);
  pair<int, int> goalPose = make_pair(3, 1);
  vector < pair<int, int> > path;
  // Call the search method
  path = OptimalPlannerTestObject.search(worldState, robotPose, goalPose);
  vector < pair<int, int> > expectedPath;
  expectedPath.push_back(robotPose);
  expectedPath.push_back(make_pair(2, 1));
  expectedPath.push_back(goalPose);
  EXPECT_EQ(expectedPath, path);
}

/**
 * @brief Check if verifyNodes method works
 */
TEST(OptimalPlanner, verifyNodesTest) {
  OptimalPlanner OptimalPlannerTestObject;
  pair<int, int> node;
  node = make_pair(1, 1);
  OptimalPlannerTestObject.visitedNodes[node] = 1;
  EXPECT_FALSE(OptimalPlannerTestObject.verifyNodes(node));
  node = make_pair(2, 2);
  OptimalPlannerTestObject.visitedNodes[node] = 0;
  EXPECT_TRUE(OptimalPlannerTestObject.verifyNodes(node));
}

/**
 * @brief Check if action method works
 */
TEST(OptimalPlanner, actionTest) {
  OptimalPlanner OptimalPlannerTestObject;
  pair<int, int> currentNode = make_pair(5, 5);
  // Run all the four actions and verify that they give the desired results
  EXPECT_EQ(make_pair(6, 5), OptimalPlannerTestObject.action(1, currentNode));
  EXPECT_EQ(make_pair(5, 6), OptimalPlannerTestObject.action(2, currentNode));
  EXPECT_EQ(make_pair(4, 5), OptimalPlannerTestObject.action(3, currentNode));
  EXPECT_EQ(make_pair(5, 4), OptimalPlannerTestObject.action(4, currentNode));
}
