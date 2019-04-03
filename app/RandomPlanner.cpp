#include "../include/RandomPlanner.h"
#include "../include/Node.h"

RandomPlanner::RandomPlanner(std::vector<std::vector<int>> map,
                             std::pair<int, int> pose,
                             std::pair<int, int> goal) {
  this->world_map = map;
  this->robot_pose = pose;
  this->goal_pose = goal;
  this->current_node.position_ = pose;
  this->x_length = int(world_map.size()) - 1;
  this->y_length = int(world_map[1].size()) - 1;
}
int RandomPlanner::randomDirection() {
  int direction;
  srand(time(0));
  return direction = rand() % 4 + 1;
}

void RandomPlanner::setGoalNode() {
  this->goal_node.value_ = this->world_map[goal_pose.first][goal_pose.second];
  this->goal_node.position_ = this->goal_pose;
}
void RandomPlanner::setStartNode() {
  this->start_node.value_ =
      this->world_map[robot_pose.first][robot_pose.second];
  this->start_node.position_ = this->robot_pose;
  //  this->start_node.parent_ = &start_node;
}

void RandomPlanner::updateLastSteps(Node a_node) {
  if (int(this->last_steps.size()) <= int(sqrt(this->max_step_number))) {
    this->last_steps.push_back(a_node.position_);
  } else {
    this->last_steps.pop_front();
    this->last_steps.push_back(a_node.position_);
  }
}
bool RandomPlanner::checkLastNSteps(Node toSearch) {
  for (auto i = this->last_steps.begin(); i != this->last_steps.end(); ++i) {
    std::pair<int, int> p = *i;
    if (toSearch.position_.first == p.first &&
        toSearch.position_.second == p.second) {
      return true;
    }
  }
  return false;
}

bool RandomPlanner::isObstacle(std::pair<int, int> to_be_checked) {
  if (to_be_checked.first > y_length || to_be_checked.first < 0 ||
      to_be_checked.second > x_length || to_be_checked.second < 0) {
    return true;
  } else if (this->world_map[to_be_checked.first][to_be_checked.second] == 1) {
    return true;
  } else
    return false;
}

/*
 * Directions
 * 1- Up
 * 2 -Left
 * 3 -DOwn
 * 4 - Right
 */
Node RandomPlanner::moveUp(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first - 1;
  tempNode.position_.second = someNode.position_.second;
  int a = tempNode.position_.first;
  int b = tempNode.position_.second;
  tempNode.value_ = this->world_map[a][b];
  //  tempNode.value_ =
  //    this->world_map[tempNode.position_.first][tempNode.position_.second];

  return tempNode;
}
Node RandomPlanner::moveLeft(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first;
  tempNode.position_.second = someNode.position_.second - 1;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];
  return tempNode;
}

Node RandomPlanner::moveDown(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first + 1;
  tempNode.position_.second = someNode.position_.second;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];
  return tempNode;
}

Node RandomPlanner::moveRight(Node someNode) {
  Node tempNode;
  // tempNode.parent_ = &someNode;
  tempNode.position_.first = someNode.position_.first;
  tempNode.position_.second = someNode.position_.second + 1;
  tempNode.value_ =
      this->world_map[tempNode.position_.first][tempNode.position_.second];
  return tempNode;
}
/*
 * Directions
 * 1- Up
 * 2 -Left
 * 3 -DOwn
 * 4 - Right
 */

void RandomPlanner::moveAround() {
  this->setStartNode();
  this->setGoalNode();
  this->current_node = this->start_node;
  this->path_.push_back(start_node.position_);
  int steps_taken = 0;
  if (this->start_node.position_ == this->goal_node.position_) {
    std::cout << "start node and goal node are same" << std::endl;
  }

  if (isObstacle(this->start_node.position_)) {
    std::cout << "start node is Obstacle" << std::endl;
  }
  if (isObstacle(this->goal_node.position_)) {
    std::cout << "goal node is Obstacle" << std::endl;
  }

  while (steps_taken <= this->max_step_number) {
    int dir = randomDirection();
    // std::cout << "random direction: " << dir << std::endl;
    bool leftFlag = false, UpFlag = false, downFlag = false, rightFlag = false;

    if (dir == 1) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.first = cu_pose.first - 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveUp(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          UpFlag = true;
        }
      }
    }
    if (dir == 2) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.second = cu_pose.second - 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveLeft(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          leftFlag = true;
        }
      }
    }
    if (dir == 3) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.first = cu_pose.first + 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveDown(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          downFlag = true;
        }
      }
    }
    if (dir == 4) {
      std::pair<int, int> cu_pose = this->current_node.position_;
      cu_pose.second = cu_pose.second + 1;
      if (!isObstacle(cu_pose)) {
        Node tempNode = this->moveRight(current_node);
        if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          downFlag = true;
        }
      }
    }

    if (!(downFlag || UpFlag || leftFlag || rightFlag)) {
      if (dir == 1) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.first = cu_pose.first - 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveUp(current_node);
          // if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          //}
        }
      }
      if (dir == 2) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.second = cu_pose.second - 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveLeft(current_node);
          //  if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          //  leftFlag = true;
          //}
        }
      }
      if (dir == 3) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.first = cu_pose.first + 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveDown(current_node);
          // if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          // downFlag = true;
          // }
        }
      }
      if (dir == 4) {
        std::pair<int, int> cu_pose = this->current_node.position_;
        cu_pose.second = cu_pose.second + 1;
        if (!isObstacle(cu_pose)) {
          Node tempNode = this->moveRight(current_node);
          // if (!checkLastNSteps(tempNode)) {
          this->current_node = tempNode;
          this->updateLastSteps(this->current_node);
          this->path_.push_back(this->current_node.position_);
          steps_taken++;
          // downFlag = true;
          // }
        }
      }
    }
    if (this->current_node.position_ == this->goal_node.position_) {
      break;
    }
    // steps_taken++;
  }
  if (this->current_node.position_ != this->goal_node.position_) {
    std::cout << "goal not found" << std::endl;
  }
}
void RandomPlanner::printPath() {
  this->moveAround();
  // std::cout << "goal node y is " << goal_pose.first << std::endl;
  // std::cout << "goal node x is " << goal_pose.second << std::endl;
  if (this->current_node.position_ == this->goal_node.position_) {
    for (int i = 0; i < int(path_.size()); i++) {
      if (i != int(path_.size()) - 1) {
        std::cout << "(" << path_[i].first << "," << path_[i].second << ") "
                  << ",";
      } else {
        std::cout << "(" << path_[i].first << "," << path_[i].second << ")"
                  << std::endl;
      }
    }
  }
}
