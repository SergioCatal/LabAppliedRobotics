#ifndef __FILLED_CONVEX_SHAPE
#define __FILLED_CONVEX_SHAPE

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#include "Shape.hpp"
#include "Segment.hpp"
#include "DubinsCurve.hpp"

/**
 * Class that represent a filled convex shape, that have an area and collision with trajectories
 */
class FilledConvexShape : public Shape {

public:

    /**
     * Empty constructor
     */
    FilledConvexShape();

    /**
     * Full constructor
     */
    FilledConvexShape(Point2f center);

    /**
     * Method to get the area of the convex shape
     *
     * @return : the area of the filled shape
     */
    virtual float getArea() const = 0;

    /**
     * Method that multiply every coordinate of the shape by the input number
     *
     * @argument scale : the number that will scale the shape
     */
    virtual void resize(float scale) = 0;

    /**
     * Method that returns true if a given point is inside the shape
     *
     * @argument p = the point to check
     *
     * @return : true if the point is inside the shape, false otherwise
     */
    virtual bool isPointInside(const Point2f &p) const = 0;

    /**
     * Method that returns true if the segment collides with the shape
     *
     * @argument seg : the segment
     *
     * @returns : true if the segment collides with the shape, false otherwise
     */
    virtual bool isSegmentColliding(const Segment &seg) const = 0;

    /**
     * Method that returns true if the given arc is colliding with the shape
     *
     * @argument center : the center of the circle
     * @argument radius : the radius of the circle
     * @argument start: the initial point of the arc
     * @argument finish : the final point of the arc
     * @argument clockwise : it is true if the arc is clockwise or counterclockwise respect to the start
     *
     * @returns : true if the arc collides with the shape, false otherwise
     */
    virtual bool isArcColliding(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const = 0;

    /**
     * Method that returns true if the dubins arc collide with the shape
     *
     * @argument da : the dubins arc to chek
     *
     * @returns : true if the dubins arc collides with the shape, false otherwise
     */
    virtual bool isDubinsArcColliding(const DubinsArc &da) const = 0;

    /**
     * Method that returns true if the dubins curve collide with the shape
     *
     * @argument dc : the dubins curve to chek
     *
     * @returns : true if the dubins curve collides with the shape, false otherwise
     */
    virtual bool isDubinsCurveColliding(const DubinsCurve &dc) const = 0;

};

#endif
