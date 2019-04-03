#include "../include/Node.h"
#include "../include/RandomPlanner.h"

int main() {
  std::vector<std::vector<int>> world_state{
      {0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 1, 0},
      {0, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 1, 0}, {0, 0, 0, 0, 0, 0}};

  std::pair<int, int> robot_pose(2, 0);
  std::pair<int, int> goal_pose(5, 5);
  RandomPlanner a;
  std::vector<std::pair<int, int>> path =
      a.search(world_state, robot_pose, goal_pose);
  return 0;
}
