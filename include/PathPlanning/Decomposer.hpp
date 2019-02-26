#ifndef __DECOMPOSER__
#define __DECOMPOSER__

#include "Map.hpp"
#include "CollisionDetector.hpp"
#include "Utility.hpp"
#include "Plotter.hpp"
#include "Cell.hpp"
#include "Graph.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;
using namespace std;

/**
 * Class that contains only static methods that can prepare the map for the decomposition and then decompose it in cell
 * Using the line-sweep algorithm
 */
class Decomposer {
private:
    /**
     * Function that eliminates the vertical lines from the obstacles of a map
     * 
     * @argument m : the map that is to modify
     */
    static void eliminateVerticalLines(Map &m);

    /**
     * Function that gets as an input a map and an offset, it clips the map to that offset, modifying the obstacles, the victims and the goal;
     * it the map and modify all the coordinates of the objects inside the map to be consistent
     */
    static void reshapeExitingPolygons(Map &m,float offset);

    /**
     * Function that from a map it generates new non convex polygons that are non overlapping
     * 
     * @argument m : the map, in order to get the obstacles
     * @argument non_overlapping_polygons : the new polygons returned that do not overlaps but can be non convex
     */
    static void eliminateOverlappingPolygons(Map &m,vector<Obstacle> &non_overlapping_polygons);

    /**
     * Function that given a map of float and segments and a target return the segment with the float
     * closer (on the positive side) to the target value, if it does not find anything it returns the 
     * top side of the map
     * 
     * @argument collisions : the map to search
     * @argument target : the value that we want to find the closest value
     * @argument m : the map, bacause if the function does not find anything it will return the top side of the map
     * 
     * @return : the Segment with a value closer (in positive) to target, otherwise the top side of the map
     */
    static Segment getUpCollision(const map<float,Segment> &collisions, float target, const Map &m);

    /**
     * Function that given a map of float and segments and a target return the segment with the float
     * closer (on the negative side) to the target value, if it does not find anything it returns the 
     * bottom side of the map
     * 
     * @argument collisions : the map to search
     * @argument target : the value that we want to find the closest value
     * @argument m : the map, bacause if the function does not find anything it will return the bottom side of the map
     * 
     * @return : the Segment with a value closer (in negative) to target, otherwise the bottom side of the map
     */
    static Segment getDownCollision(const map<float,Segment> &collisions, float target,const Map &m);
    

public:

    /**
     * Function that will prepare the map for the line sweep algorithm, clipping the obstacles, clipping the map,
     * eliminating vertical lines and getting the new polyogns that do not overlaps
     * 
     * @argument clippingQuantity : the value of the clipping of the map and of the obstacles
     * @argument m : the map that will be modified
     * @argument non_overlapping_polygons : the new polygons returned that do not overlaps but can be non convex
     */
    static void prepareMap(float clippingQuantity, Map &m, vector<Obstacle> &non_overlapping_polygons);

    /**
     * Function that implements the line sweep algorithm, and that returns the graph of the navigable points and connections
     * between them in the map
     * 
     * @ærgument pg : the graph where the new points and their connections will be saved
     * @argument m : the map to decompose
     * @argument non_overlapping_polygons : the polygons of the map in the non overlapping version, but that are non convex
     * @argument points_per_cell : number of points that every cell will create
     */
    static void decompose(PointsGraph &pg, Map m, vector<Obstacle> non_overlapping_polygons,int points_per_cell);

};

#endif
