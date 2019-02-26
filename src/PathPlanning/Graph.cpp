#include "Graph.hpp"
#include<queue>
#include<algorithm>

// add a point to the graph
int PointsGraph::addPoint(cv::Point2f p){
  int ret;
  auto it = indexes.find(p);
  if(it == indexes.end()){
    ret = insertNode(p);
    //std::cout << "Inserting node (" << p.x << "," << p.y << ") in POS: " << ret << std::endl;
  }
  else{
    ret = it->second;
    //std::cout << "Found node (" << p.x << "," << p.y << ")in POS: " << ret << std::endl;
  }

  return ret;
}

// remove a point to the graph
void PointsGraph::removeNode(cv::Point2f p){
  auto it = indexes.find(p);
  if(it != indexes.end()){
    nodes[it->second].clearNeighbors();
  }
}

// add a connection to the graph
void PointsGraph::addEdge(int p1, int p2){
  nodes[p1].addNeighbor(p2);
  nodes[p2].addNeighbor(p1);
}

// add a connection to the graph
void PointsGraph::addEdge(cv::Point2f p1, cv::Point2f p2){
  addEdge(addPoint(p1), addPoint(p2));
}

// add a point to the graph
int PointsGraph::insertNode(cv::Point2f p){
  int ret = nodes.size();
  nodes.push_back(PointNode(p));
  indexes.insert(std::pair<cv::Point2f, int>(p, ret));
  return ret;
}

// print method
void PointsGraph::print(){
  for(unsigned int i = 0; i < nodes.size(); i++){
    std::cout << "Node " << i << ": (" << nodes[i].getX() << "," << nodes[i].getY() << ")" << std::endl << "\t";
    for(unsigned int k = 0; k < nodes[i].getNeighbors().size(); k++){
      std::cout << nodes[i].getNeighbors()[k] << " ";
    }
    std::cout << std::endl;
  }
}

struct DistancedNode{
  int node;
  float distance;
  DistancedNode(){}
  DistancedNode(int n, float d) : node(n), distance(d){}
  int operator() (const DistancedNode& p1, const DistancedNode& p2)
  {
      return (p1.distance > p2.distance);
  }
};

// get the square of the distance between two points
float PointsGraph::distancePP(int p1, int p2){
  float dx = nodes[p2].getX() - nodes[p1].getX();
  float dy = nodes[p2].getY() - nodes[p1].getY();
  return (dx*dx + dy*dy);
}

// compute the shortest path from one point to the other
void PointsGraph::shortestPath(cv::Point2f start_point, cv::Point2f destination, std::vector<cv::Point2f> &path){
  auto it1 = indexes.find(start_point);
  auto it2 = indexes.find(destination);
  if(it1 != indexes.end() && it2 != indexes.end()){
    shortestPath(it1->second, it2->second, path);
  } else {
    path.clear();
  }

}

// compute the shortest path from one point to the other
void PointsGraph::shortestPath(int start_point, int destination, std::vector<cv::Point2f> &path){
  int V = nodes.size();
  path.clear();
  std::priority_queue<DistancedNode, std::vector<DistancedNode>, DistancedNode> pq;
  int* parent = new int[V];
  float* distance = new float[V];
  bool* added = new bool[V];

  for(int i = 0; i < V; i++){
    parent[i] = -1;
    added[i] = false;
  }

  pq.push(DistancedNode(start_point, 0));
  distance[start_point] = 0;
  parent[start_point] = start_point;
  DistancedNode next_node;

  while(!pq.empty()){
    next_node = pq.top();
    pq.pop();
    if(next_node.node == destination)
      break;

    if(!added[next_node.node]){
      added[next_node.node] = true;
      auto neighbors = nodes[next_node.node].getNeighbors();
      for(unsigned int n : neighbors){
        float newd = distance[next_node.node] + distancePP(next_node.node, n);
        if(parent[n] == -1 || (parent[n] != -1 && distance[n] > newd)){
          distance[n] = newd;
          parent[n] = next_node.node;
          pq.push(DistancedNode(n, newd));
        }
      }
    }
  }

  int tmp = destination, count = 1;
  while(parent[tmp] != tmp){
    count++;
    tmp = parent[tmp];
  }

  path.resize(count--);
  tmp = destination;
  int sn = 0;
  while(parent[tmp] != tmp){
    path[count - (sn++)] = nodes[tmp].getPoint();
    tmp = parent[tmp];
  }
  path[count - (sn++)] = nodes[tmp].getPoint();

  delete[] parent;
  delete[] added;
  delete[] distance;
}

// get the nodes
std::vector<PointNode>& PointsGraph::getNodes() { 
  return this->nodes;
}
