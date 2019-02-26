#ifndef __OBSTACLE__
#define __OBSTACLE__

#include "Polygon.hpp"
#include "BoundingBox.hpp"

class Obstacle : public Polygon, public BoundedShape {
private:
    BoundingBox boundingBox;

public:
    /**
     * Empty constructor, it assigns to every attribute the value 0
     */
    Obstacle();

    /**
     * Full constructor
     *
     * @argument center : the center point of the obstacle
     * @argument vertices : the vector containing the vertices of the obstacle
     */
    Obstacle(Point2f center, vector<Point2f> vertices) ;

    /**
     * Method that prints the obstacle on the terminal
     */
    void print() const;

    /**
     * Function that calculates a object containing the two points of the obstacle that coicide with
     * the lower x and y and the highest x and y
     */
    void calculateBoundingBox();

    /**
     * Function that enlarge the edges of the obstacle
     *
     * @argument offset : the dimension of the enlargment
     */
    void clipperEdges(float offset);

    /**
     * Function that multiply every coordinate of the obstacle by the input number
     *
     * @argument scale : the number that will multiply every coordinate of the obstacle
     */
    void resize(float scale) override;

    /**
     * Function that returns true if the segment composed by two points touches the bounding box of the obstacle
     *
     * @argument p1 : starting point of the segment
     * @argument p2 : final point of the segment
     *
     * @returns : true if the segment touches the bounding box of the obstacle, false otherwise
     */
    bool isSegmentCollidingWithBoundingBox(const Point2f &p1, const Point2f &p2) const;

    /**
     * Function that returns true if the given arc is touching the bounding box of the obstacle
     *
     * @argument center : the center of the circle
     * @argument radius : the radius of the circle
     * @argument start: the initial point of the arc
     * @argument finish : the final point of the arc
     *
     * @returns : true if the arc touches the bounding box of the obstacle, false otherwise
     */
    bool isArcCollidingWithBoundingBox(const Point2f &center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const;

    /**
     * Function that returns true if the dubins arc collide with the bounding box of the obstacle
     *
     * @argument da : the dubins arc to chek
     *
     * @returns : true if the dubins arc touches the bounding box of the obstacle, false otherwise
     */
    bool isDubinsArcCollidingWithBoundingBox(const DubinsArc &da) const;

    /**
     * Function that returns true if the dubins arc collide with the obstacle
     *
     * @argument da : the dubins arc to chek
     *
     * @returns : true if the dubins arc touches the obstacle, false otherwise
     */
    bool isDubinsCurveCollidingWithBoundingBox(const DubinsCurve &dc) const;

    /**
     * Method to get the bounding box of the obstacle
     *
     * @return : the bounding box of the obstacle
     */ 
    BoundingBox& getBoundingBox() override;

    /**
     * Method to check if the bb is intersecting a DubinsCurve
     *
     * @argument dc: the DubinsCurve to check
     *
     * @returns : true if it collides, false if it doesn't
     */
    bool bbIntersectsDubinsCurve(DubinsCurve &dc) override;

    /**
     * Function that returns true if the point is inside the polygon
     *
     * @argument p : the point to check
     *
     * @returns : true if the point is inside the polygon, false otherwise
     */
    bool isPointInside(const Point2f &p) const override;
};

#endif
