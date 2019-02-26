#ifndef AABBTREE_HPP
#define AABBTREE_HPP
#include "Obstacle.hpp"
#include "DubinsCurve.hpp"
#include "BoundingBox.hpp"
#include "Obstacle.hpp"

// TODO implement and comment it

class AABBTree {
public:
  AABBTree(){}
  AABBTree(std::vector<Obstacle> &_obstacles);
  ~AABBTree();
  bool check_collision(DubinsCurve &dc);
  static bool compareCentersX(const Obstacle *o1, const Obstacle *o2);
  static bool compareCentersY(const Obstacle *o1, const Obstacle *o2);
  std::vector<BoundedShape*>& getBoundingBoxes();

private:
  std::vector<Obstacle*> obstacles;
  std::vector<BoundedShape*> nodes;
  bool check_collision_recursive(DubinsCurve &dc, int ind);

  BoundedShape* mergeBoundingBoxes(int begin, int end);
  void TopDownBBTree(int tree, int begin, int end);
  int leftChild(int parent);
  int rightChild(int parent);
  int parent(int child);
  bool isLeaf(int node);
};

#endif
