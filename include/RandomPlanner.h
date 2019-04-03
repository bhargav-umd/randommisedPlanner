#pragma once
#include "./Node.h"
#include <algorithm>
#include <deque>
#include <iostream>
#include <queue>
#include <time.h>
#include <utility>
#include <vector>
class RandomPlanner {
private:
  int max_step_number = 100;
  std::deque<std::pair<int, int>> last_steps;
  std::vector<std::vector<int>> world_map;
  std::pair<int, int> robot_pose;
  std::pair<int, int> goal_pose;
  std::vector<std::pair<int, int>> path_;
  Node start_node;
  Node goal_node;
  Node current_node;
  int dir;
  int y_length;
  int x_length;

public:
  RandomPlanner(std::vector<std::vector<int>>, std::pair<int, int>,
                std::pair<int, int>);
  //  void setYLength();
  // void setXLength();
  Node moveUp(Node);
  Node moveLeft(Node);
  Node moveDown(Node);
  Node moveRight(Node);
  void setGoalNode();
  void setStartNode();
  void getGoal();
  void getPose();
  bool isObstacle(std::pair<int, int>);
  int randomDirection();
  void updateLastSteps(Node);
  bool checkLastNSteps(Node);
  bool canWeGo(Node);
  void printPath();
  void moveAround();
};
