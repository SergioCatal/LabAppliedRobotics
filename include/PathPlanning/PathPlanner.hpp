#ifndef __PATHPLANNER__
#define __PATHPLANNER__

#include "CollisionDetector.hpp"
#include "Graph.hpp"
#include "DubinsSolver.hpp"
#include "Plotter.hpp"
#include "path.h"

/**
 * Class for the robotic path planning of a robot that uses dubins Curve
 */
class PathPlanner {
private:
    CollisionDetector cd;
    PointsGraph pg;
    Map m;
    float victim_bonus;
    float robot_velocity;
    float k;

    /**
     * Calculate the arrival angle based on the goal in the map
     * 
     * @return : the arrival angle
     */
    float calculateGoalArrivalAngle() ;

    /**
     * Find the graph points that are closest to the start and end points
     * 
     * @argument start : the first point and in the returned value the point closer to this
     * @argument end : the second point and in the returned value the point closer to this
     */
    void getClosestPointsToGraph(Point2f &start, Point2f &end);

    /**
     * Calculate the lenght of a path constituded of points 
     * 
     * @argument path : the path to calculate the lenght of
     * 
     * @return : the lenght of the path
     */
    float getGraphPathLenght(const vector<Point2f> &path);

    /**
     * Calculate the score of a path composed by DubinsCurve
     * 
     * @argument path : the DubinsCurve path
     * @argument n_victims_rescued : the number of victims rescued by this path
     * 
     * @return : the score of the path
     */
    float calculatePathScore(const vector<DubinsCurve> &path, int n_victims_rescued);

    /**
     * Take a random point inside the victim that does not collide with obstacles and does not exit from the map
     *
     * @ærgument v : the victim 
     * @argument obstacles : the obstacles of the map
     * @argument mapLimits : the bounding box representing the map limits
     * 
     * @return : a point inside the victim that does not collide with obstacles and does not exit from the map
     */
    Point2f getPointInsideCircle(const Victim &v,const vector<Obstacle>&obstacles, const BoundingBox &mapLimits);

    /**
     * Get int the returned points a vector of vector containing the best order or visiting each victim for any number of victims
     * 
     * @argument points_to_visit : the points to visit
     * @argument distances : a matrix with the distance of every point to every other
     * @argument returned_points : a vector of vector containing the best order or visiting each victim for any number of victims
     */
    void getVictimsOrder(vector<Point2f> points_to_visit, const vector<vector<float>> &distances, vector<vector<Point2f>> &return_points);

    /**
     * Method to eliminate the furtherst victim(from the start point and the last point) from the matrix of the distances
     * 
     * @argument matrix : a matrix with the distance of every point to every other
     * @argument victim_to_eliminate : the number ind the matrix of the victim to eliminate
     * 
     * @return : the new matrix of distances
     */
    vector<vector<float>> eliminateFurthestVictimFromMatrix(vector<vector<float>> matrix, int &victim_to_eliminate);
public:
    /**
     * Empty constructor
     */
    PathPlanner();

    /**
     * Full constructor
     * 
     * @argument m : the map 
     * @argument pg : the pointGraph of free points and paths
     * @argument robot_k : the robot curvature
     * @argument robot_velocity : the robot velocity in mm/s
     * @argument : the bonus in seconds for taking a victim
     */
    PathPlanner(const Map &m, const PointsGraph &pg, float robot_k, float robot_velocity, float victim_bonus);

    /**
     * Method that transforms a path with DubinsCurves in a Path object
     * 
     * @argument curves : the DubinsCurve path
     * @argument path_lenght : the lenght of the DubinsCurve path
     * @argument path : the path object that will be filled with information from the DubinsCurve path
     */
    void dubinsCurvesToPoints(const vector<DubinsCurve> &curves, double path_length, Path &path);

    /**
     * Method that is recursive; it takea as an input a list of points and two index, and try to go from the starting point to the end
     * point with a dubins curve without collisions. If this succedes the curve is added to the solution, otherwise this mehtod will call itself
     * two times, splitting the path. if there are no more points to split the function it will return false
     * 
     * @argument start_index : the index in the vector of points of the starting point
     * @arugment start_angle : the start angle of the starting point
     * @argument end_index : the index in the vector of points of the final point
     * @argument end_angle : pointer to the end angle, if NULL will be chosen randomly
     * @argument graph : the vector of points where to find start and end point
     * @argument solution : the vector to append the correct DubinsCurve
     * 
     * @return : true if it was possible to find a solution, false otherwise
     */
    bool recursive(int start_index, float start_angle, int end_index, float* end_angle, const vector<Point2f> &graph, vector<DubinsCurve> *solution);

    /**
     * Method that will generate a path from a point to the other, utilizing the graph and the recursive function
     * 
     * @argument start_point : the initial point of the path
     * @argument start_angle : the angle to start the path
     * @argument end_point: the final point 
     * @argument end_angle : the end angle of the path, if it is NULL will be chosen randomly
     * @argument pathreturned : the path find will be appended here
     * 
     * @return : true if it was possible to find a path, false otherwise
     */
    bool getPath(Point2f start_point, float start_angle, Point2f end_point, float* end_angle, vector<DubinsCurve> *pathreturned);

    /**
     * General path palnner algorithm, it takes a initial point and angle, a list of points to go and an and angle, and return the path
     * 
     * @argument start_angle : the angle to start the path
     * @argument end_angle : the final agle the path 
     * @argument points_to_visit : the list of points to visit
     * @argument solution : the path computed will be appended here
     * 
     * @return : true if it was possible to find a path, false otherwise
     */
    bool generalPathPlanner(float start_angle, float end_angle, const vector<Point2f> &points_to_visit, vector<DubinsCurve> *solution);

    /**
     * Find a path that visits all the victims in order and goes to the goal
     * 
     * @argument start_point : the starting point
     * @argument start_angle : the starting angle
     * @argument returnedPath : the path will be appended here
     * 
     * @return : true if it was possible to find a path, false otherwise
     */
    bool saveVictimsPath(Point2f start_point, float start_angle,vector<DubinsCurve> &returnedPath);

    /**
     * Find a path that gives the best score. This path can pick any number of victims in the map
     * 
     * @argument start_point : the starting point
     * @argument start_angle : the starting angle
     * @argument returnedPath : the path will be appended here
     * @argument path_score : the score of the returned path
     * 
     * @return : true if it was possible to find a path, false otherwise
     */
    bool missionPlan(Point2f start_point, float start_angle,vector<DubinsCurve> &returnedPath, float &path_score);

    /**
     * Calculate the lenght of a path constituded of DubinsCurve 
     * 
     * @argument path : the path to calculate the lenght of
     * 
     * @return : the lenght of the path
     */
    static float calculatePathLenght(const vector<DubinsCurve> &path);

};

#endif
