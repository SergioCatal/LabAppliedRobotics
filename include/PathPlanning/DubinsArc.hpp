#ifndef __DUBINSARC__
#define __DUBINSARC__

#include "Shape.hpp"
#include "Segment.hpp"
#include "path.h"

/**
 * Class that represents a Dubins Arc
 */
class DubinsArc : public Shape{
private:
	double x0;
	double y0;
	double th0;
	double k;
	double l;
	double xf;
	double yf;
	double thf;

public:

	/**
	 * Empty constructor, it sets everything to 0
	 */
	DubinsArc();

	/**
	 * Full constructor
	 *
	 * @argument x0 : initial x of the arc
	 * @argument y0 : initial y of the arc
	 * @argument th0 : initial angle of the arc
	 * @argument k : the curvature of the arc (the inverse of the radius)
	 * @argument l : the lenght of the arc
	 * @argument xf : the final x of the arc
	 * @argument yf : the final y of the arc
	 * @argument thf : the final angle of the arc
	*/
	DubinsArc(double x0, double y0, double th0, double k, double l, double xf, double yf, double thf);

	/**
	 * Constructor with start parameters
	 *
	 * @argument x0 : initial x of the arc
	 * @argument y0 : initial y of the arc
	 * @argument th0 : initial angle of the arc
	 * @argument k : the curvature of the arc (the inverse of the radius)
	 * @argument l : the lenght of the arc
	 */
	DubinsArc(double x0, double y0, double th0, double k, double l);

	/**
	 * Method that adds sampled points of this dubins arc to the input path
	 *
	 * @argument overlength : length of the part of the previous arc which has not been inserted in the vector.
	 * @argument last_one : true if this is the last arc in the path. If yes, the last point of the curve is added to the path
	 * @argument path : the path to which the points are added
	 * @return : the overlength of this arc
	 */
	double addPoints(double overlength, Path &path) const;

	/**
	 * Method that returns the point with curvlinear coordinate s on this arc
	 *
	 * @argument s : curvilinear coordinate of the point to retrieve
	 * @argument pose: the pose of the robot in that point. This function has to modify the values of x, y and theta inside pose
	 */
	void getPoint(double s, Pose& pose) const;

	/**
	 * Function that returns the final x
	 *
	 * @return : the final x
	 */
	double getXf() const;

	/**
	 * Function that returns the final y
	 *
	 * @return : the final y
	 */
	double getYf() const;

	/**
	 * Function that returns the final angle
	 *
	 * @return : the final angle
	 */
	double getThf() const;

	/**
	 * Function that returns the lenght l of the arc
	 *
	 * @return : lenght l of the arc
	 */
	double getL() const;

	/**
	 * Function that returns the curvature k of the arc
	 *
	 * @return : the curvature k of the arc
	 */
	double getK() const;

	/**
	 * Function that returns the initial x
	 *
	 * @return : the initial x
	 */
	double getX0() const;

	/**
	 * Function that returns the initial y
	 *
	 * @return : the initial y
	 */
	double getY0() const;

	/**
	 * Function that returns the initial angle
	 *
	 * @return : the initial angle
	 */
	double getTh0() const;

	/**
     * Print method
     */
    void print() const;

	/**
	 * Function that given a dubins arc returns a general arc
	 *
	 * @argument center : at the and of the computation inside there will be the center of the circle where the arc is lying
	 * @argument radius : at the and of the computation inside there will be the radius of the circle where the arc is lying
	 * @argument start_point : at the and of the computation inside there will be the start point of the arc
	 * @argument start_point : at the and of the computation inside there will be the end point of the arc
	 * @argument clockwise : @argument start_point : at the and of the computation inside there will be true if the arc is in clockwise direction, false otherwise
	 *
	 */
	void getArc(Point2f &center, float &radius, Point2f &start_point, Point2f &end_point, bool &clockwise);

};


#endif
