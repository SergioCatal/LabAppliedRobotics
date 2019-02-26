#ifndef __BOUNDING_BOX__
#define __BOUNDING_BOX__

#include "Segment.hpp"
#include "FilledConvexShape.hpp"

/**
 * Class that represent a BoundingBox with 4 numbers
 */
class BoundingBox : public FilledConvexShape, public BoundedShape {
private:
    float min_x;
    float min_y;
    float max_x;
    float max_y;

public:
    /**
     * Public empty constructor
     */
    BoundingBox();

    /**
     * Public full constructor
     *
     * @argument min_x : the smallest x value of the BoundingBox
     * @argument min_y : the smallest y value of the BoundingBox
     * @argument max_x : the biggest x value of the BoundingBox
     * @argument max_y : the biggest y value of the BoundingBox
     */
    BoundingBox(float min_x, float min_y, float max_x, float max_y);

    /**
     * Method that modifies the box to include another box
     *
     * @argument other: other bounding box to include in this one
     **/
    void merge(BoundingBox &other);

    /**
     * Method that multiply every coordinate of the BoundingBox by the input number
     *
     * @argument scale : the number that will scale the BoundingBox
     */
    void resize(float scale) override;

    /**
     * Function that returns true if the segment collides with the BoundingBox box
     *
     * @argument seg : the segment
     *
     * @returns : true if the segment collides the BoundingBox, false otherwise
     */
    bool isSegmentColliding(const Segment &seg) const override;

     /**
     * Function that returns true if the given arc collides with the BoundingBox
     *
     * @argument center : the center of the circle arc
     * @argument radius : the radius of the circle arc
     * @argument start: the initial point of the arc
     * @argument finish : the final point of the arc
     * @argument clockwise : it is true if the arc is clockwise or counterclockwise respect to the start
     *
     * @returns : true if the arc touches the BoundingBox, false otherwise
     */
    bool isArcColliding(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const override;

    /**
     * Function that returns the area of the BoundingBox
     *
     * @return : the area of the BoundingBox
     */
    float getArea() const override;

    /**
     * Function that prints the information of the BoundingBox
     */
    void print() const override;

    /**
     * Function that returns true if the dubins arc collides with the BoundingBox
     *
     * @argument da : the dubins arc to chek
     *
     * @returns : true if the dubins arc collides the BoundingBox, false otherwise
     */
    bool isDubinsArcColliding(const DubinsArc &da) const override; 

    /**
     * Function that returns true if a dubins curve is colliding with the BoundingBox
     *
     * @argument dc : the dubins curve to check if it collides
     *
     * @return : true is the dubins curve collide with the BoundingBox, false otherwise
     */
    bool isDubinsCurveColliding(const DubinsCurve &dc) const override;

    /**
     * Function that returns true if the point is inside the BoundingBox
     *
     * @argument p : the point to check
     *
     * @returns : true if the point is inside the BoundingBox, false otherwise
     */
    bool isPointInside(const Point2f &p) const override;

    /**
     * Method to get the minimum x
     *
     * @return : the minimum x
     */
    float getMinX() const;

    /**
     * Method to get the minimum y
     *
     * @return : the minimum y
     */
    float getMinY() const;

    /**
     * Method to get the maximum x
     *
     * @return : the maximum x
     */
    float getMaxX() const;

    /**
     * Method to get the maximum y
     *
     * @return : the maximum y
     */
    float getMaxY() const;

    /**
     * Method that returns true if the dubins arc is touching one of the boundaries of the bounding box
     * is different from isDubinsArcColliding() because it does not check is the dubins arc is inside
     * the bounding box
     * 
     * @argument da : dubins arc to check
     * 
     * @return true if the dubins arc is touching the boundaries of the bounding box, false otherwise
     */
    bool isDubinsArcTouching(const DubinsArc &da) const;

    /**
     * Method that returns the four segments that compose the bounding box
     * 
     * @return : the segments of the bounding box
     */
    vector<Segment> getSegments();

    // TO comment Sergio
    BoundingBox& getBoundingBox() override;
    bool bbIntersectsDubinsCurve(DubinsCurve &dc) override;
};



#endif
