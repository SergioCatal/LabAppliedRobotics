#include "Circle.hpp"

// Empty constructor
Circle::Circle() : FilledConvexShape(){
    radius=0;
}

// Full constructor
Circle::Circle(Point2f _center, double _radius) : FilledConvexShape(_center){
    radius = _radius;
}

// Function that returns the radius of the circle
double Circle::getRadius() const {
    return radius;
}

// Set function for the radius
void Circle::setRadius(float radius) {
    this->radius = radius;
}

// Resize the center and the radius
void Circle::resize(float scale){
    center.x = center.x * scale;
    center.y = center.y * scale;

    radius = radius * scale;
}

// Print function
void Circle::print() const {
    cout << "Circle center " <<  center << " radius " << radius << endl;
}

// Return the area of the circle, Pi*radius^2
float Circle::getArea() const {
    return M_PI*radius*radius;
}

// Function for the collision with a segment
bool Circle::isSegmentColliding(const Segment &seg) const{
    // get the center of the circle where the line lies
    float xc = center.x;
    float yc = center.y;

    // get the segment coordinates
    float x1 = seg.getP1().x;
    float x2 = seg.getP2().x;
    float y1 = seg.getP1().y;
    float y2 = seg.getP2().y;

    float p1 = 2 * x1 * x2; // intermediate variables
    float p2 = 2 * y1 * y2;
    float p3 = 2 * xc * x1;
    float p4 = 2 * xc * x2;
    float p5 = 2 * yc * y1;
    float p6 = 2 * yc * y2;

    // c1, c2 and c3 are the coefficients of the equation c1*t^2 + c2*t +c3 = 0
    float c1 = x1*x1 + x2*x2 - p1 + y1*y1 + y2*y2 - p2;
    float c2 = -2*x2*x2 + p1 - p3 + p4 - 2*y2*y2 + p2 - p5 + p6;
    float c3 = x2*x2 - p4 + xc*xc + y2*y2 - p6 + yc*yc - radius*radius;

    float delta = c2*c2 - 4 * c1 * c3; // calculate the delta of the equation
    vector<float> t_vector = {};
    //cout << "delta : " << delta << endl;

    // delta is less than 0, no intersection of this circle with the segment
    if(delta < 0) {
        //cout << "Segment has no intersection with the circle" << endl;
        return false;
    } else if( delta*delta <= 0.000001) { // only one solution case
        float t1 = (-c2) / (2*c1);

        if(t1 >= 0 && t1 <= 1) {    // if t1 is in the t[0,1] interval the solution is in the segment
            return true;
        }

    } else { // two solutions
        float t1 = (-c2 + sqrt( delta ) ) / (2*c1);
        float t2 = (-c2 - sqrt( delta) ) / (2*c1);
        if(t1 >= 0 && t1 <= 1) { // if t1 is in the t[0,1] interval the solution is in the segment
            return true;
        }
        if(t2 >= 0 && t2 <= 1) { // if t2 is in the t[0,1] interval the solution is in the segment
            return true;
        }
        //cout << "t1: " << t1 << " t2: " << t2 << endl;
    }
    // segment do not touch the circle, but can be inside if one of its limit point is inside the circle
    return isPointInside(seg.getP1());
}

// Function for the collision with an arc
bool Circle::isArcColliding(const Point2f &arc_center, float radius,const Point2f &start, const Point2f &finish, bool clockwise) const{
    Segment seg = Segment(center,arc_center);

    float theta_start = atan2((start.y - arc_center.y),(start.x - arc_center.x));
    float theta_finish = atan2((finish.y - arc_center.y),(finish.x - arc_center.x));
    theta_start = Utility::mod2pi(theta_start);
    theta_finish = Utility::mod2pi(theta_finish);

    if(seg.getLength() > radius + this->radius) {
        if(clockwise) {

        } else {
            
        }
        return false;
    } 
    // TODO: implement better this part


    // there are no collision between the circle and the arc, but it can still be inside
    return isPointInside(start);
}

// Function for the collision with a dubins arc
bool Circle::isDubinsArcColliding(const DubinsArc &da) const{

    float k = da.getK();

    // straight line case
    if(k == 0) {
        Point2f p1 = Point2f(da.getX0(), da.getY0() ) ;
        Point2f p2 = Point2f(da.getXf(), da.getYf() ) ;

        return this->isSegmentColliding(Segment(p1,p2));
    // arc case
    } else {
        float radius = abs(1 / k);
        Point2f p1 = Point2f(da.getX0(), da.getY0() ) ;
        Point2f p2 = Point2f(da.getXf(), da.getYf() ) ;
        Point2f center;
        if( da.getK() < 0) {
            float xc = cos( da.getTh0() - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( da.getTh0() - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = Point(xc + da.getX0(),yc + da.getY0());
        } else {
            float xc = cos( da.getTh0() + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( da.getTh0() + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = Point(xc + da.getX0(),yc + da.getY0());
        }

        return this->isArcColliding(center,radius,p1,p2,true);

    }


}

// Function for the collision with a dubins curve
bool Circle::isDubinsCurveColliding(const DubinsCurve &dc) const{
    for(unsigned int i = 0; i < 3; i++) {
        DubinsArc da = dc.getArc(i);
        if(this->isDubinsArcColliding(da)) {
            return true;
        }
    }
    return false;
}

// Function that returns true if the point is inside the circle
bool Circle::isPointInside(const Point2f &p) const{
    Segment seg = Segment(this->getCenter(),p);

    if(seg.getLength() > this->radius) {
        return false;
    } else {
        return true;
    }
}
