#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#define M_2PI 2*M_PI

using namespace cv;
using namespace std;

/**
 * General utility class that include all the function useful
 */
class Utility{
public:

  /**
   * Function that calculate the sinc of an input value x
   * 
   * @argument x : the input value
   * 
   * @return : the sinc value of x
   */
  static double sinc(double x);

  /**
   * Function that calculate the module 2 Pi of an input angle
   * 
   * @argument ang : the input angle 
   * 
   * @return : the input angle in module 2 Pi
   */
  static double mod2pi(double ang);

  /**
   * Private method that converts a float point into a int point
   * 
   * @argument pin : the input float point
   * 
   * @return : the input point trasformed into a integer point
   */
  static Point fromFloatPointToIntPoint(const Point2f &pin);

  /**
   * Private method that converts a vecto of float points into a vector of int points
   * 
   * @argument pin : the input vector of float points
   * 
   * @return : the vector of input points trasformed into a vector of integer points
   */
  static vector<Point> fromFloatPointsToIntPoints(const vector<Point2f> &pin);

  /**
   * Function that given a point it substitute the original y value with the value y_max - y
   * it is usefoul because opencv invert the y of all points when drawing
   * 
   * @argument pin : the input point
   * @argument y_max : the maximum value of y allowed
   * 
   * @return : the input point with the y coordinated substituted by the value y_max - y
   */
  static Point invertYPoint(const Point &pin, int y_max);

  /**
   * Return the point with the greatest x
   * 
   * @argument p1 : first point
   * @argument p2 : second point
   * 
   * @return : true if p1 has grater x coordinate that p2
   */
  static bool greaterXPoint2f(const Point2f &p1, const Point2f &p2);

  /**
   * Return the point with the greatest y
   * 
   * @argument p1 : first point
   * @argument p2 : second point
   * 
   * @return : true if p1 has grater y coordinate that p2
   */
  static bool greaterYPoint2f(const Point2f &p1, const Point2f &p2);

  /** 
   * Function that computer the mean of a vector of points, that is basically its center
   * 
   * @argument points : the points to compute the center
   * 
   * @return : the mean point
   */
  static Point2f computePointsCenter(const vector<Point2f> &points);
  
};

#endif
