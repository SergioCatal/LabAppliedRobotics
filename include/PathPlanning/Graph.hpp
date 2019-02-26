#ifndef GRAPH_HPP
#define GRAPH_HPP
#include<iostream>//DEBUG
#include<vector>
#include<unordered_map>
#include<opencv2/opencv.hpp>

/**
 * Class that represents a point in a graph
 */
class PointNode{
public:
  PointNode(cv::Point2f p) : point(p){
  }

  /**
   * Method to add a neighbor to the point
   * 
   * @argument n : the number of the neighbor in the graph
   */
  void addNeighbor(int n){
    neighbors.push_back(n);
  }

  /**
   * Method to get the neighbor of this point
   * 
   * @return : the list of the neigboors
   */
  const std::vector<int>& getNeighbors() const {
    return neighbors;
  }

  /**
   * Method to clean the neighboors
   */
  void clearNeighbors(){
    neighbors.clear();
  }

  /**
   * Method to return the x of the point
   * 
   * @return : the x of the point
   */
  inline float getX() const {
    return point.x;
  }
  /**
   * Method to return the y of the point
   * 
   * @return : the y of the point
   */
  inline float getY() const {
    return point.y;
  }

  /**
   * Method to return the point
   * 
   * @return : the point
   */
  inline const cv::Point2f& getPoint() const{
    return point;
  }

private:
  cv::Point2f point;
  std::vector<int> neighbors;
};

/**
 * Struct to crete an hash of points
 */
struct point_hash
{
	std::size_t operator() (const cv::Point2f &pair) const
	{
		return std::hash<float>()(pair.x) ^ std::hash<float>()(pair.y);
	}
};

/**
 * Class that represents a graph of points
 */
class PointsGraph {
public:

  /**
   * Method that adds a point to the graph
   * 
   * @argument p : the point to add
   * 
   * @return : the int position of the point in the graph
   */
  int addPoint(cv::Point2f p);

  /**
   * Method to remove a point from the graph
   * 
   * @ærgument p : the point to remove
   */
  void removeNode(cv::Point2f p);

  /**
   * Method to add a connection between two points in the graph
   * 
   * @argument p1 : the index of the first point
   * @argument p2 : the index of the second point
   */
  void addEdge(int p1, int p2);

  /**
   * Method to add a connection between two points
   * 
   * @argument p1 : the first point
   * @argument p2 : the second point
   */
  void addEdge(cv::Point2f p1, cv::Point2f p2);

  /**
   * Method to get the shortest path from one point to the other
   * 
   * @argument p1 : starting point
   * @argument p2 : final point
   * @argument path : at the end of the function the pah of points will be here
   */
  void shortestPath(int p1, int p2, std::vector<cv::Point2f> &path);

  /**
   * Method to get the shortest path from one point to the other
   * 
   * @argument p1 : starting point
   * @argument p2 : final point
   * @argument path : at the end of the function the pah of points will be here
   */
  void shortestPath(cv::Point2f p1, cv::Point2f p2, std::vector<cv::Point2f> &path);

  /**
   * Method that returns all the nodes in the graph
   * 
   * @return : the vector of nodes of the graph
   */
  std::vector<PointNode>& getNodes();

  /**
   * Print method
   */
  void print();

private:
  std::vector<PointNode> nodes;
  std::unordered_map<cv::Point2f, int, point_hash> indexes;

  /**
   * Method that adds a point to the graph
   * 
   * @argument p : the point to add
   * 
   * @return : the int position of the point in the graph
   */
  int insertNode(cv::Point2f p);

  /**
   * Method to calculate the distance of two points
   */
  float distancePP(int p1, int p2);
};

#endif
