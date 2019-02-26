#ifndef __DUBINSCURVE__
#define __DUBINSCURVE__

#include "DubinsArc.hpp"
#include "Shape.hpp"

/**
 * Class that represents a Dubins curve as three Dubins arcs
 */
class DubinsCurve : public Shape{
    private:
    DubinsArc arc1;
    DubinsArc arc2;
    DubinsArc arc3;

    public:

    /**
     * Empty constructor
     */
    DubinsCurve();

    /**
     * Full constructor
     *
     * @argument x0 : the initial x
     * @argument y0 : the initial y
     * @argument th0 : the initial angle
     * @argument l1 : the first arc lenght
     * @argument l2 : the second arc lenght
     * @argument l3 : the third arc lenght
     * @argument k1 : the first arc curvature
     * @argument k2 : the second arc curvature
     * @argument k3 : the third arc curvature
     */
    DubinsCurve(double x0, double y0, double th0, double l1, double l2, double l3, double k1, double k2, double k3);

    /**
     * Mathod that returns the requested Dubins arc
     *
     * @argument pos : 1 for the first arc, 2 for the second and 3 for the third
     *
     * @return : the requested Dubins arc
     */
    const DubinsArc& getArc(int pos) const;

    /**
     * Method that returns the total lenght of the curve
     *
     * @return : the total lenght of the curve
     */
    double getTotalLenght() const;

    /**
     * Method that adds sampled points of this dubins curve to the input path
     *
     * @argument overlength : length of the final part of the previous curve which has not been inserted in the vector. The first point added will be added at curvilinear length = sampling_length - overlength (if overlength is 0, then the first point will be at 0) in order to preserve the distance between points in the path
     * @argument path : the path to which the points are added
     * @return : the overlength of this curve
     */
    double addPoints(double overlength, Path &path) const;

    /**
     * Method that return the initial x of the first arc
     *
     * @return :  the initial x of the first arc
     */
    double getX0() const;

    /**
     * Method that return the initial y of the first arc
     *
     * @return :  the initial y of the first arc
     */
    double getY0() const;

     /**
     * Method that return the initial angle of the first arc
     *
     * @return :  the initial angle of the first arc
     */
    double getTh0() const;

    /**
     * Method that return the initial x of the second arc, which is the final x of the first arc
     *
     * @return :  the initial x of the second arc, which is the final x of the first arc
     */
    double getX1() const;

    /**
     * Method that return the initial y of the second arc, which is the final y of the first arc
     *
     * @return :  the initial y of the second arc, which is the final y of the first arc
     */
    double getY1() const;

    /**
     * Method that return the initial angle of the second arc, which is the final angle of the first arc
     *
     * @return :  the initial angle of the second arc, which is the final angle of the first arc
     */
    double getTh1() const;

    /**
     * Method that return the initial x of the third arc, which is the final x of the second arc
     *
     * @return :  the initial x of the third arc, which is the final x of the second arc
     */
    double getX2() const;

    /**
     * Method that return the initial y of the third arc, which is the final y of the second arc
     *
     * @return :  the initial y of the third arc, which is the final y of the second arc
     */
    double getY2() const;

    /**
     * Method that return the initial y of the third arc, which is the final y of the second arc
     *
     * @return :  the initial y of the third arc, which is the final y of the second arc
     */
    double getTh2() const;

    /**
     * Method that return the final x of the third arc
     *
     * @return :  the final x of the third arc
     */
    double getXf() const;

    /**
     * Method that return the final x of the third arc
     *
     * @return :  the final x of the third arc
     */
    double getYf() const;

    /**
     * Method that return the final y of the third arc
     *
     * @return :  the final y of the third arc
     */
    double getThf() const;

    /**
     * Print method
     */
    void print() const;

    /**
     * Operator < in order to sort DubinsCurve based on their lenght
     * 
     * @argument other : the other DubinsCurve to confront with
     * 
     * @return : true if this curve lenght is minor that the other DubinsCurve total lenght
     */
    bool operator < (const DubinsCurve& other) const;

};

#endif
