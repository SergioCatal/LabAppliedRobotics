#ifndef __MAP_GENERATOR__
#define __MAP_GENERATOR__

#include "Map.hpp"

/**
 * Class that generates random sintetical maps
 */
class MapGenerator {
public:

    /**
     * Function to generate a new random map
     * 
     * @argument width : the with of the map to be generated
     * @argument height : the height of the map to be generated
     * @argument n_victims : the number of victims of the map
     * @argument victims_radius : the radius of the victims 
     * @argument n_obstacles : the number of obstacles 
     * @argument obstacles_radius : the radius of the obstacles 
     * 
     * @return : the new generated map
     */
    static Map getRandomMap(int width, int height,int n_victims, float victims_radius, int n_obstacles, float obstacles_radius);

    /**
     * Function to get a random goal
     * 
     * @argument width : the width of the map
     * @argument height : the height of the map
     * @argument obstacle_radius : the radius of the obstacles
     * 
     * @return : an obstacle representing the new goal
     */
    static Obstacle getRandomGoal(int width, int height,float obstacles_radius);

    /**
     * Function to get the vertices of an obstacle
     * 
     * @argument center : the center of the polygon
     * @argument obstacle_radius : the radius of the obstacle
     * 
     * @return : generated vertices
     */
    static vector<Point2f> getRandomPolygon(Point2f center, float obstacles_radius);

};

#endif
