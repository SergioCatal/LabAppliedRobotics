#include"Utility.hpp"
#include<cmath>

// Mathod that compute the module 2 Pi of an angle
double Utility::mod2pi(double angle){
  while(angle < 0)
    angle += M_2PI;
  while(angle >= M_2PI)
    angle -= M_2PI;
  return angle;
}

// method that compute the sinc of a value
double Utility::sinc(double x) {
	double result = 0;

	if (std::abs(x) < 0.002) {
		result = 1.0 - x*x / 6.0 * (1.0 - x*x / 20.);
	}
	else {
		result = std::sin(x) / x;
	}

	return result;
}

// Method that converts a float point into a int point
Point Utility::fromFloatPointToIntPoint(const Point2f &pin) {
  return Point(pin.x,pin.y);
}

// Method that converts a vector of float points into a vector of int points
vector<Point> Utility::fromFloatPointsToIntPoints(const vector<Point2f> &pin) {
  vector<Point> vec_int = {};
  for(unsigned int i = 0; i < pin.size();i++) {
    vec_int.push_back(Utility::fromFloatPointToIntPoint(pin[i]));
  }
  return vec_int;
}

// Invert y function
Point Utility::invertYPoint(const Point &pin, int y_max) {
	return Point(pin.x , y_max - pin.y);
}

bool Utility::greaterXPoint2f(const Point2f &p1,const Point2f &p2) {
  return p1.x < p2.x;
}

bool Utility::greaterYPoint2f(const Point2f &p1, const Point2f &p2) {
  return p1.y < p2.y;
}

Point2f Utility::computePointsCenter(const vector<Point2f> &points) {

  float sum_x = 0;
  float sum_y = 0;

  for(unsigned int i = 0; i < points.size();i++) {
    sum_x += points[i].x;
    sum_y += points[i].y;
  }

  sum_x = sum_x / points.size();
  sum_y = sum_y / points.size();

  return Point2f(sum_x,sum_y);

}
