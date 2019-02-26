#ifndef __CIRCLE__
#define __CIRCLE__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#include "FilledConvexShape.hpp"
#include "Segment.hpp"

/**
 * Class that represent a circle with a center and a radius.
 */
class Circle : public FilledConvexShape {
    protected:
    double radius;

    public:

    /**
     * Empty constructor, it assigns to every attribute the value 0
     */
    Circle();

    /**
     * Full constructor
     *
     * @argument _center : the center of the circle
     * @argument _radius : the radius of the circle
     */
    Circle(Point2f _center, double _radius);

    /**
     * Method that returns the radius of the circle
     *
     * @return: the radius of the circle
     */
    double getRadius() const;

    /**
     * Method that permits to set the radius
     *
     * @argument radius : the new radius
     */
    void setRadius(float radius);

    /**
     * Function that multiply the center and the radius by the input number
     *
     * @argument scale : the number that will multiply the center and the radius
     */
    void resize(float scale) override;

    /**
     * Methods that print the circle on the terminal
     */
    void print() const override;

    /**
     * Function that returns the area of the circle
     *
     * @return : the area of the circle
     */
    float getArea() const override;

    /**
     * Method that returns true if the segment collides with the circle, virtual method in FilledConvexShape
     *
     * @argument seg : the segment
     *
     * @returns : true if the segment collides with the circle, false otherwise
     */
    bool isSegmentColliding(const Segment &seg) const override;

    /**
     * Function that returns true if the given arc is touching the circle
     *
     * @argument center : the center of the circle
     * @argument radius : the radius of the circle
     * @argument start: the initial point of the arc
     * @argument finish : the final point of the arc
     *
     * @returns : true if the arc touches the circle, false otherwise
     */
    bool isArcColliding(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const override;

    /**
     * Function that returns true if the dubins arc collide with the circle
     *
     * @argument da : the dubins arc to chek
     *
     * @returns : true if the dubins arc touches the circle, false otherwise
     */
    bool isDubinsArcColliding(const DubinsArc &da) const override;

    /**
     * Function that returns true if a dubins curve is colliding with the circle
     *
     * @argument dc : the dubins curve to check if it collides
     *
     * @return : true is the dubins curve collide with the circle, false otherwise
     */
    bool isDubinsCurveColliding(const DubinsCurve &dc) const override;

    /**
     * Function that returns true if the point is inside the circle
     *
     * @argument p : the point to check
     *
     * @returns : true if the point is inside the circle, false otherwise
     */
    bool isPointInside(const Point2f &p) const override;

};

#endif
