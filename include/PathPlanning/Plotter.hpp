#ifndef __PLOTTER__
#define __PLOTTER__

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;
#include "Map.hpp"
#include "DubinsCurve.hpp"
#include "Utility.hpp"
#include "Graph.hpp"

/**
 * Class that permits to print various shapes on the same image
 */
class Plotter {
private:
    Mat img;
    int circle_thickness;
    int polygon_thickness;
    int line_thickness;
    Scalar background_color;
    Scalar default_poly_color;
    Scalar default_circle_color;
    Scalar default_line_color;


    /**
     * Method that tries to draw an arrow given a point and a direction
     *
     * @argument p : the starting point
     * @argument theta : the angle that gives the direction to the arrow
     */
    void drawArrow(Point p, float theta);

public:
    /**
    * Empty constructor, it creates a 1024x1024 empty black image
    */
    Plotter();

    /**
    * Partial constructor, it creates a black image of the specified dimension
    *
    * @argument rows : the rows of the image
    * @argument columns : the columns of the image
    */
    Plotter(int rows, int culumns);

    /**
    * Empty full constructor
    *
    * @argument rows : the rows of the image
    * @argument columns : the columns of the image
    * @argument color : the background color of the image
    */
    Plotter(int rows, int culumns , Scalar color);

    /**
     * Method that draws an obstacle on the internal image
     *
     * @argument o : the obstacle to draw
     */
    void drawObstacle(const Obstacle &o);

    /**
     * Method that draws an obstacle on the internal image
     *
     * @argument o : the obstacle to draw
     * @argument color : the coor of the obstacle
     */
    void drawObstacle(const Obstacle &o, Scalar color);

    /**
     * Method that draws a vector of obstacles on the internal image
     *
     * @argument obstacles : the vector of obstacles to draw
     */
    void drawObstacles(const  vector<Obstacle> &obstacles);

    /**
     * Method that draws a polygon on the internal image
     *
     * @argument p : the polygon to draw
     */
    void drawPolygon(const Polygon &p);

    /**
     * Method that draws a bounding box on the internal image
     *
     * @argument bb : the bounding box to draw
     */
    void drawBoundingBox(const BoundingBox &bb);

    /**
     * Method that draws a circle on the internal image
     *
     * @argument c : the circle to draw
     */
    void drawCircle(const Circle &c);

    /**
     * Method that draws a circle on the internal image, that permits to specify the color
     *
     * @argument c : the circle to draw
     * @argument color : the color of the circle
     */
    void drawCircle(const Circle &c,Scalar color);

    /**
     * Method that draws a victim on the internal image
     *
     * @argument v : the victim to draw
     */
    void drawVictim(const Victim &v);

    /**
     * Method that draws a segment on the internal image
     *
     * @argument seg : the segment to draw
     */
    void drawSegment(const Segment &seg);

    /**
     * Method that draws a segment on the internal image
     *
     * @argument seg : the segment to draw
     * @argument color: the color of the segment
     */
    void drawSegment(const Segment &seg,Scalar color);

    /**
     * Method that draws a dubins arc on the internal image
     *
     * @argument da : the dubins arc to draw
     */
    void drawDubinsArc(const DubinsArc &da);

    /**
     * Method that draws a dubins curve on the internal image
     *
     * @argument dc : the dubins curve to draw
     */
    void drawDubinsCurve(const DubinsCurve &dc);

    /**
     * Method that draws a vector of dubins curve on the internal image
     *
     * @argument dc : the dubins curves to draw
     */
    void drawDubinsCurves(const vector<DubinsCurve> &dc);

    /**
     * Method that draws an entire map on the internal image
     *
     * @argument map : the map to draw
     */
    void drawMap(Map map);

    /**
     * Method that shows the internal image
     */
    void show();

    /**
     * Method that returns the internal image
     *
     * @return : the internal image of the plotter
     */
    Mat getImg();

    /**
     * Method that draws a graph of points
     *
     * @argument pg : the PointsGraph to draw
     */
    void drawPointsGraph(PointsGraph pg);

    /**
     * Method that draws a path from the clipper library
     *
     * @argument Paths : the paths to draw
     */
    void drawClippingLibPaths(const ClipperLib::Paths &paths);

    /**
     * Method that eliminate the current image and replace it with the default plotter image ( a black 1024x1024 image)
     *
     */
    void clear();

};


#endif
