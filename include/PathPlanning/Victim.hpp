#ifndef __VICTIM__
#define __VICTIM__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#include "Circle.hpp"

/** Class that represents a victim as a circle with a number inside
 *
 */
class Victim : public Circle {
    protected:
    int number;

    public:
    /**
     * Empry constructor
     */
    Victim ();

    /**
     * Full constructor
     *
     * @argument number : the number of the victim
     * @argument center : the center of the victim circle
     * @argument radius : the radius of the victim circle
     */
    Victim(int number, Point2f center, float radius);

    public:
    /**
     * Method that returns the number associated with the circle
     *
     * @return: the number inside the circle
     */
    int getNumber() const ;

     /**
     * Method that permits to set the number inside the circle
     *
     * @argument number : the new number
     */
    void setNumber(int number);

    /**
     * Methods that print the information about the victim, is an override of the print of the superclass
     */
    void print() const;

    /**
     * Operator < definition, in order to sort a victims array based on their number
     */
    bool operator< (const Victim& other);

};


#endif
