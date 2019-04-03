#include "../include/Node.h"

Node::Node(){};

Node::Node(int value, std::pair<int, int> position) {
  this->value_ = value;
  this->position_ = position;
  //  this->parent_ = parent;
}
