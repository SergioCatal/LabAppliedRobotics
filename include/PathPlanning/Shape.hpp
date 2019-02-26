#ifndef __SHAPE__
#define __SHAPE__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/**
 * Class that represent a general shape with a point representing its center
 * It is a virtual class
 */
class Shape {
protected:
    Point2f center;

public:
    /**
     * Empty constructor
     */
    Shape();

    /**
     * Empty destructor
     */
    virtual ~Shape(){};

    /**
     * Full constructor
     *
     * @argument center : the center of the shape
     */
    Shape(Point2f center);

    /**
     * Method that returns the center of the shape
     *
     * @return : the center of the shape
     */
    Point2f getCenter() const;

    /**
     * Method that permits to set the center of the shape
     *
     * @argument center : the new center of the shape
     */
    void setCenter(Point2f center);

    /**
     * Print method, it is virtual in order that the class can be virtual
     */
    virtual void print() const = 0;

};

// TODO SERGIO
class BoundingBox;
class DubinsCurve;
class BoundedShape{
public:
  virtual BoundingBox& getBoundingBox() = 0;
  virtual bool bbIntersectsDubinsCurve(DubinsCurve &dc) = 0;
  virtual ~BoundedShape(){}
};

#endif
