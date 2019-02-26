#ifndef __DUBINSSOLVER__
#define __DUBINSSOLVER__

#include"DubinsCurve.hpp"

class DubinsSolver{
public:
  struct DubinsProblemParameters{
    double x0, y0, th0, xf, yf, thf, k_max;
  };
  struct DubinsProblemScaledParameters{
    double th0, thf, k_max, lambda;
  };

  /**
   * Function that returns the shortest dubins curve from a starting point and an starting angle to a final point and an final angle
   * 
   * @argument x0 : initial x coordinate
   * @argument y0 : initial y coordinate
   * @argument th0 : initial angle
   * @argument xf : final x coordinate
   * @argument yf : final y coordinate
   * @argument thf : final angle coordinate
   * @argument k_max : the curvature of the robot
   * @argument results : an array of pointers to DubinsCurve that will be popolated by the function
   * 
   * @return : the index of the shortest curve in the results array
   */
  static int shortestPath(double x0, double y0, double th0, double xf, double yf, double thf, double k_max, DubinsCurve *results[6]);

  /**
   * Function that will free the memory occupie by the curves for the shortest path
   * 
   * @ærgument results : the curves to free
   */
  static void freeCurves(DubinsCurve *results[6]);

private:
  /**
   * Function that will scale the coordinates
   * 
   * @argument lambda : the scale
   * @argument sc_s1 : the first coordinate to scale
   * @argument sc_s2 : the second coordinate to scale
   * @argument sc_s3 : the third coordinate to scale
   */
  static void scaleFromStandard(double lambda, double &sc_s1, double &sc_s2, double &sc_s3);

  /**
   * Function that implements the LeftStraightLeft DubinsArc
   */
  static bool LSL(const DubinsProblemScaledParameters &sc_params, double &s1, double &s2, double &s3);

  /**
   * Function that implements the RightStraightRight DubinsArc
   */
  static bool RSR(const DubinsProblemScaledParameters &sc_params, double &s1, double &s2, double &s3);

  /**
   * Function that implements the LeftStraightRight DubinsArc
   */
  static bool LSR(const DubinsProblemScaledParameters &sc_params, double &s1, double &s2, double &s3);

  /**
   * Function that implements the RightStraightLeft DubinsArc
   */
  static bool RSL(const DubinsProblemScaledParameters &sc_params, double &s1, double &s2, double &s3);

  /**
   * Function that implements the RightLeftRight DubinsArc
   */
  static bool RLR(const DubinsProblemScaledParameters &sc_params, double &s1, double &s2, double &s3);

  /**
   * Function that implements the LeftRightLeft DubinsArc
   */
  static bool LRL(const DubinsProblemScaledParameters &sc_params, double &s1, double &s2, double &s3);

  // TODO Sergio
  typedef bool (*curve_type_solution)(const DubinsProblemScaledParameters &, double &, double &, double &);
  static const curve_type_solution methods[6];
  static const int curvatures[6][3];

};

#endif
