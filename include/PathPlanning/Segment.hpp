#ifndef __SEGMENT__
#define __SEGMENT__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Utility.hpp"
#include "Shape.hpp"

using namespace cv;
using namespace std;

/**
 * Class that represent a segment as two points
 */
class Segment : public Shape{
private:
    Point2f p1;
    Point2f p2;
public:
    /**
     * Empty constructor
     */
    Segment();

    /**
     * Full constructor (p1 and p2 must be different)
     *
     * @argument p1 : the starting point of the segment
     * @argument p2 : the finish point of the segment
     */
    Segment(Point2f p1,Point2f p2);

    /**
     * Method that returns the lenght of the segment
     *
     * @return : the lenght of the segment
     */
    float getLength() const;

    /**
     * Method that returns the maximum x value of the segment
     *
     * @return : the maximum x value of the segment
     */
    float getMaxX() const;

    /**
     * Method that returns the maximum y value of the segment
     *
     * @return : the maximum y value of the segment
     */
    float getMaxY() const;

    /**
     * Method that returns the minimum x value of the segment
     *
     * @return : the minimum x value of the segment
     */
    float getMinX() const;

    /**
     * Method that returns the minimum y value of the segment
     *
     * @return : the minimum y value of the segment
     */
    float getMinY() const;

    /**
     * Function that returns the m parameter of the line given by the two points of the segment.
     * M is the parameter in the general line equation y = m*x + q
     *
     * @return : the m parameter of the line given by the two points of the segment, if the segment is vertical the return value is Nan
     */
    double getMLine() const ;

    /**
     * Function that returns the q parameter of the line given by the two points of the segment.
     * Q is the parameter in the general line equation y = m*x + q
     *
     * @return : the q parameter of the line given by the two points of the segment, if the segment is vertical the return value is Nan
     */
    double getQLine() const ;

    /**
     * Print method
     */
    void print() const;

    /**
     * Method that returns true if the segment is colliding with another segment
     *
     * @argument seg : segment to check
     *
     * @return : true if the two segments collide
     */
    bool isCollidingWithSegment(const Segment &seg) const;

    /**
     * Method that returns the point of collision with another segment
     *
     * @argument seg : segment to check
     *
     * @return : the point of collision with the other segment
     */
    Point2f getCollisionPointWithSegment(const Segment &seg) const;

    /**
     * Method that returns true if the segment is colliding with an arc
     *
     * @argument arc_center: the point of the arc to check
     * @argument radius : the radius of the arc to check
     * @argument start : the start point of the arc to check
     * @argument finish : the last point of the arc to check
     * @argument clockwise : it is true if the arc is clockwise or counterclockwise respect to the start
     *
     * @return : true if the segment and the arc collide
     */
    bool isCollidingWithArc(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const;

    /**
     * Method that returns the first point of the segment
     *
     * @return : the first point of the segment
     */
    Point2f getP1() const;

    /**
     * Method that returns the last point of the segment
     *
     * @return : the last point of the segment
     */
    Point2f getP2() const;

    /**
     * Method that permits to set the first point of the segment
     *
     * @argument p1 : the new first point of the semgent
     */
    void setP1(Point2f p1);

    /**
     * Method that permits to set the last point of the segment
     *
     * @argument p2 : the new last point of the semgent
     */
    void setP2(Point2f p2);

    /**
     * Comparison operator, it check is the initial and final points of the two segments are the same
     *
     * @return : true if the two segments are have the same Points P1 and P2, false otherwise
     */
    bool operator ==(const Segment &other) const;
};


#endif
