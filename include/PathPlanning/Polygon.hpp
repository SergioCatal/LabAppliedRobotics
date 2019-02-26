#ifndef __POLYGON__
#define __POLYGON__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#include "Segment.hpp"
#include "DubinsCurve.hpp"
#include "FilledConvexShape.hpp"
#include "clipper.hpp"

/**
 * Class that represent a polygon as a vector of points with a center
 * It is derived from the class shape
 */
class Polygon : public FilledConvexShape{

protected:
    vector<Point2f> vertices;
    vector<Segment> segments;

public:
    /**
     * Empty constructor, it assigns to every attribute the value 0
     */
    Polygon();

    /**
     * Full constructor
     *
     * @argument center : the center point of the polygon
     * @argument vertices : the vector containing the vertices of the polygon
     */
    Polygon(Point2f center, vector<Point2f> vertices) ;

    /**
     * Function that returns the number of vertices of the polygon
     *
     * @return : the number of vertices of the polygon
     */
    int getNVertices();

    /**
     * Method that return the vertices of the polygon
     *
     * @return: the vertices of the polygon
     */
    vector<Point2f> getVertices() const;

    /**
     * Method that return the area of the polygon
     *
     * @return: the area of the polygon
     */
    float getArea() const override;

    /**
     * Method that returns the segments of the polygon
     *
     * @return : the segment list of the polygon
     */
    vector<Segment> getSegments() const;

    /**
     * Method that prints the polygon on the terminal
     */
    void print() const override;

    /**
     * Copy constructor override
     *
     * @argument other : the source polygon to copy
     */
    Polygon(const Polygon& other);

    /**
     * Assignment operator override
     *
     * @argument other : the source polygon to copy
     */
    Polygon& operator= (const Polygon& other);

    /**
     * Function that returns the list of segments of the polygon
     *
     * @returns : a vector containing the segments objects that form the polygon
     */
    void calculateSegments();

    /**
     * Function that enlarge the edges of the polygon
     *
     * @argument offset : the dimension of the enlargment
     */
    void clipperEdges(float offset);

    /**
     * Function that multiply every coordinate of the polygon by the input number
     *
     * @argument scale : the number that will multiply every coordinate of the polygon
     */
    void resize(float scale) override;

    /**
     * Function that returns true if the segment composed by two points touches the polygon
     *
     * @argument seg : the segment
     *
     * @returns : true if the segment touches the polygon, false otherwise
     */
    bool isSegmentColliding(const Segment &seg) const override;

    /**
     * Function that returns true if the given arc is touching the polygon
     *
     * @argument center : the center of the circle
     * @argument radius : the radius of the circle
     * @argument start: the initial point of the arc
     * @argument finish : the final point of the arc
     * @argument clockwise : it is true if the arc is clockwise or counterclockwise respect to the start
     *
     * @returns : true if the arc touches the polygon, false otherwise
     */
    bool isArcColliding(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const override;

    /**
     * Function that returns true if the dubins arc collide with the polygon
     *
     * @argument da : the dubins arc to chek
     *
     * @returns : true if the dubins arc touches the polygon, false otherwise
     */
    bool isDubinsArcColliding(const DubinsArc &da) const override;

    /**
     * Function that returns true if a dubins curve is colliding with the polygon
     *
     * @argument dc : the dubins curve to check if it collides
     *
     * @return : true is the dubins curve collide with the polygon, false otherwise
     */
    bool isDubinsCurveColliding(const DubinsCurve &dc) const override;

    /**
     * Function that returns true if the point is inside the polygon
     *
     * @argument p : the point to check
     *
     * @returns : true if the point is inside the polygon, false otherwise
     */
    bool isPointInside(const Point2f &p) const override ;

};

#endif
