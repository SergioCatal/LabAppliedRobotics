#ifndef ___MAP__
#define ___MAP__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#include "Victim.hpp"
#include "Obstacle.hpp"

/**
 * Class that represent the map of the board. It has the sidth and the height of the board.
 * It has a vector of obstacles that are representd by polygons.
 * It has also a vecotr of victims and the goal object.
 */
class Map {
    int width;
    int height;
    vector<Obstacle> obstacles;
    vector<Victim> victims;
    Obstacle goal;

    public:

    /**
     * Empty costructor, it sets null or 0 all the attributes
     */
    Map();

    /**
     * Full constructor
     */
    Map(int width, int height, Obstacle goal, vector<Obstacle> obstacles, vector<Victim> victims);

    /**
     * Function that returns the width of the board represented by the map
     *
     * @return : the width of the map
     */
    int getWidth() const;

    /*
     * Function that returns the width of the board represented by the map
     *
     * @return : the height of the map
     */
    int getHeight() const;

    /**
     * Function that returns the goal
     *
     * @return : the goal of the map
     */
    Obstacle getGoal() const;

    /**
     * Function that returns the obstacles vector
     *
     * @return : the obstacles vector of the map
     */
    vector<Obstacle>& getObstacles();

    /**
     * Function that permits to set the obstacles of the map
     *
     * @argument obstacles : the new obsacle
     */
    void setObstacles(const vector<Obstacle> &new_obstacles);

    /**
     * Function that returns the obstacles vector
     *
     * @return : the obstacles vector of the map
     */
    vector<Victim>& getVictims();

    /*
     * Function that print every information in the map
     */
    void print();

    /**
     * Function that prints on the file system the object as a txt file
     *
     * @argument filename: the filename of the file where te output will be written
     *
     * The format is the following:
     *
     * Width Height
     * Number_of_obstacles+number_of_victims
     * X_center_goal Y_center_goal Goal_vertice_1 Goal_vertice_2 Goal_vertice_3 Goal_vertice_4
     * X_center_shape Y_center_shape Num_vertices Vertice1_shape ...
     *
     * the last row was a generis shape description, with first the coordinates of the center,
     * then how many vertices has the shape and then all the coordinates of the shape.
     * Is the shape is a circle it has 0 vertices, and after the 0 there is the radius and the number
     *
     */
    void printOnFileSystem(string filename);

    /*
     * Function that reas a txt file in the format of printOnFileSystem() and it loads the content of that
     * file in the object, overriding its original content
     *
     * @argument filename: the filename of the input file
     */
    void readFile(string filename);

    /**
     * Function that resize the entire map
     *
     * @argument scale : the scale of the resizing
     */
    void resize(float scale);

    /**
     * Method that clip the edges of all obstacles inside the map (not the goal)
     *
     * @argument offset : the quantity to clip the polygons
     *
     */
    void clipObstaclesEdges(float offset);

};

#endif
