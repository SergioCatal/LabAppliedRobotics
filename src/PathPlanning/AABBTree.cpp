#include"AABBTree.hpp"
#include<cmath>

//IF I want to use the obstacles for some feature in the end, I must save a pointer to the obstacle instead of a pointer to its bounding box
void AABBTree::TopDownBBTree(int tree, int begin, int end) {
  assert(end - begin >= 0);

  if (end == begin){
    //std::cout << "Reached the leaf "<< tree << ": done" << std::endl;
    //nodes[tree] = (FilledConvexShape*)obstacles[begin]; //ENABLED ONLY IF we want to insert the pointer to the bounding boxes as a
  } else {
    //ChooseAxis
    BoundingBox &bb_tree = nodes[tree]->getBoundingBox();

    char axis = (((bb_tree.getMaxX() - bb_tree.getMinX()) > (bb_tree.getMaxY() - bb_tree.getMinY())) ? 'x' : 'y');

    //SortPointers in obstacles
    std::sort(obstacles.begin() + begin, obstacles.begin() + end, ((axis == 'x') ? compareCentersX : compareCentersY));

    //TakeMedian
    int middle = (begin + end) / 2;


    int left_child = leftChild(tree);
    int right_child = left_child + 1;
    assert(left_child < nodes.size() && right_child < nodes.size());

    nodes[left_child] = mergeBoundingBoxes(begin, middle);
    nodes[right_child] = mergeBoundingBoxes(middle + 1, end);
    //std::cout << "Left child of " << tree << ":"; nodes[left_child]->getBoundingBox().print(); std::cout << std::endl;
    //std::cout << "Right child of " << tree << ":"; nodes[right_child]->getBoundingBox().print(); std::cout << std::endl;

    TopDownBBTree(left_child, begin, middle);
    TopDownBBTree(right_child, middle + 1, end);
  }
}

AABBTree::AABBTree(std::vector<Obstacle> &_obstacles) : obstacles(_obstacles.size()), nodes(std::pow(2, (int)(std::log2((float)_obstacles.size())) + 2) - 1){
  int n = _obstacles.size();
  for(int i = 0; i < n; i++)
    obstacles[i] = &(_obstacles[i]);

  std::fill(nodes.begin(), nodes.end(), nullptr);
  nodes[0] = mergeBoundingBoxes(0, n-1);
  TopDownBBTree(0, 0, n-1);
}


AABBTree::~AABBTree(){
  for(int i = 0; i < nodes.size(); i++){
    if(nodes[i] && dynamic_cast<BoundingBox*>(nodes[i]))
        delete nodes[i];

  }
}

bool AABBTree::check_collision_recursive(DubinsCurve &dc, int ind){
  std::cout << "Before IF"  << std::endl;
  if(nodes[ind]->bbIntersectsDubinsCurve(dc)){
    std::cout << "After IF" << std::endl;

    if(isLeaf(ind)){
      std::cout << "LEAF returning true" << std::endl;
      return true;
    }else{
      std::cout << "NON-LEAF"<< std::endl;
      bool ret = check_collision_recursive(dc, leftChild(ind));
      ret |= check_collision_recursive(dc, rightChild(ind));
      return ret;
    }
  } else {
    std::cout << "After ELSE" << std::endl;

    std::cout << "NOT Colliding with ";
    nodes[ind]->getBoundingBox().print();
    std::cout << endl;
  }
  return false;
}

bool AABBTree::check_collision(DubinsCurve &dc){
  return check_collision_recursive(dc, 0);
}



bool AABBTree::compareCentersX(const Obstacle *o1, const Obstacle *o2){
  return o1->getCenter().x < o2->getCenter().x;
}

bool AABBTree::compareCentersY(const Obstacle *o1, const Obstacle *o2){
  return o1->getCenter().y < o2->getCenter().y;
}


BoundedShape* AABBTree::mergeBoundingBoxes(int begin, int end){
  if(begin == end){
    return obstacles[begin];
  }else{
    BoundingBox &first_bb = obstacles[begin]->getBoundingBox();
    BoundingBox* ret = new BoundingBox(first_bb.getMinX(), first_bb.getMinY(), first_bb.getMaxX(), first_bb.getMaxY());

    for(int i = begin + 1; i <= end; i++)
      ret->merge(obstacles[i]->getBoundingBox());

    return ret;
  }
}

std::vector<BoundedShape*>& AABBTree::getBoundingBoxes(){
  return nodes;
}


int AABBTree::leftChild(int parent){
  return parent * 2 + 1;
}

int AABBTree::rightChild(int parent){
  return parent * 2 + 2;
}

int AABBTree::parent(int child){
  return (child - 1) / 2;
}

bool AABBTree::isLeaf(int node){
  return (leftChild(node) >= nodes.size() || (!nodes[leftChild(node)]));
}
