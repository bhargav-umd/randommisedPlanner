#pragma once
#include <iostream>
#include <utility>

struct Node {
  Node();
  Node(int, std::pair<int, int>);
  std::pair<int, int> position_;
  // Node *parent_;
  int value_;
};
