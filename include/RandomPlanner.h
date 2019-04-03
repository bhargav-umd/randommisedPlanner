/* Copyright (C)
 * 2019 - Bhargav Dandamudi
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the 'Software'), to deal in the Software without
 * restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so,subject to
 * the following conditions:
 * The above copyright notice and this permission notice shall
 * be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED ''AS IS'', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM,OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/**
 * @file RandomPlanner.h
 * @brief  Main Random Discrete Planner Class with all declarations
 * @author Bhargav Dandamudi
 * @version 1
 * @date 2019-04-03
 */
#pragma once
#include "./Node.h"
#include <algorithm>
#include <deque>
#include <iostream>
#include <queue>
#include <time.h>
#include <utility>
#include <vector>
/* ----------------------------------------------------------------*/
/**
 * @brief The random planner tries to find a path to the goal by randomly moving
in the environment (only orthogonal moves are legal). If the planner can not
find an acceptable solution in less than max_step_number, the search should
fail. The random planner, while being erratic, has a short memory, and it will
never attempt to visit a cell that was visited in the last sqrt(max_step_number)
steps except if this is the only available option.
 */
/* ----------------------------------------------------------------*/
class RandomPlanner {
private:
  int max_step_number = 100; // as in problem statement, can be any +ve integer
  std::deque<std::pair<int, int>>
      last_steps; // Queue Memory of steps taken so far
  std::vector<std::vector<int>> world_map;
  std::pair<int, int> robot_pose;
  std::pair<int, int> goal_pose;
  std::vector<std::pair<int, int>> path_;
  Node start_node;
  Node goal_node;
  Node current_node;
  int dir;      // direction inwhich robot will move
  int y_length; // max height of map in y direction
  int x_length; // max width of map in x direction

public:
  /* ----------------------------------------------------------------*/
  /**
   * @brief Default constructor to take map, starting position and goal
   * position
   * of robot
   * @param std::vector<std::vector> world map
   * @param std::pair starting position
   * @param std::pair goal position
   */
  /* ----------------------------------------------------------------*/
  RandomPlanner(std::vector<std::vector<int>>, std::pair<int, int>,
                std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  to Move in up direction, updates current node
   *
   * @param Node present node location
   *
   * @return Update Node to move in upper block
   */
  /* ----------------------------------------------------------------*/
  Node moveUp(Node);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  TO move in left location
   *
   * @param Node node to be moved
   *
   * @return updated Node
   */
  /* ----------------------------------------------------------------*/
  Node moveLeft(Node);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  TO move in +y direction
   *
   * @param Node node to be udpated
   *
   * @return updated Node
   */
  /* ----------------------------------------------------------------*/
  Node moveDown(Node);
  /* ----------------------------------------------------------------*/
  /**
   * @brief   to move in +x direction
   *
   * @param Node node to be updated
   *
   * @return updated node
   */
  /* ----------------------------------------------------------------*/
  Node moveRight(Node);
  /* ----------------------------------------------------------------*/
  /**
   * @brief to fetch goal node location
   */
  /* ----------------------------------------------------------------*/
  void getGoal();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  to get current pose of the robot
   */
  /* ----------------------------------------------------------------*/
  void getPose();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  To check if robot is in defined boundary
   *
   * @param std::pair robot coordinates
   *
   * @return true if its blocked/obstacle, else false
   */
  /* ----------------------------------------------------------------*/
  bool isObstacle(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  to give random direction everytime we run this function
   *          in range of 1 to 4
   * @return random direction as {1,2,3,4}
   */
  /* ----------------------------------------------------------------*/
  int randomDirection();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  update the last steps queue to maintain memory
   *
   * @param Node location of new location to be updated in memory
   */
  /* ----------------------------------------------------------------*/
  void updateLastSteps(Node);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  To check if input node is in memory or not
   *
   * @param Node
   *
   * @return true if the node is in memory
   */
  /* ----------------------------------------------------------------*/
  bool checkLastNSteps(Node);
  bool checkLastNPositions(std::pair<int, int>);
  /* ----------------------------------------------------------------*/
  /**
   * @brief  to print the robot path if found
   */
  /* ----------------------------------------------------------------*/
  void printPath();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  To move the robot randomly as per given direction until
   *          max number of steps are out or goal is reached.
   */
  /* ----------------------------------------------------------------*/
  void moveAround();

  /* ----------------------------------------------------------------*/
  /**
   * @brief  Sets goal Node as per given value,location
   */
  /* ----------------------------------------------------------------*/
  void setGoalNode();
  /* ----------------------------------------------------------------*/
  /**
   * @brief  Sets Start Node as per given value and location
   */
  /* ----------------------------------------------------------------*/
  void setStartNode();

  std::vector<std::pair<int, int>> findNeighbors(std::pair<int, int>);
  bool allNeighborsInMemory(std::pair<int, int>);
};
