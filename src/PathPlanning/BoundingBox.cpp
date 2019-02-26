#include "BoundingBox.hpp"

// Empty constructor 
BoundingBox::BoundingBox() : FilledConvexShape() {
    this->min_x = 0;
    this->min_y = 0;
    this->max_x = 0;
    this->max_y = 0;
}

// Full constructor
BoundingBox::BoundingBox(float _min_x, float _min_y, float _max_x, float _max_y)
    : FilledConvexShape( Point2f( (_max_x-_min_x)/2 + _min_x, (_max_y-_min_y)/2 + _min_y  ) ){
    this->min_x = _min_x;
    this->min_y = _min_y;
    this->max_x = _max_x;
    this->max_y = _max_y;
}

// copy constructor
void BoundingBox::merge(BoundingBox &other){
  this-> min_x = std::min(other.getMinX(), this->min_x);
  this-> min_y = std::min(other.getMinY(), this->min_y);
  this-> max_x = std::max(other.getMaxX(), this->max_x);
  this-> max_y = std::max(other.getMaxY(), this->max_y);
}

// Print function
void BoundingBox::print() const {
    cout << "BoundingBox from (" << min_x << "," << min_y << ") to (" << max_x << "," << max_y << ")" << endl;
}

// Get the area of the BoundingBox
float BoundingBox::getArea() const {
    float a = (this->max_x - this->min_x); // the lenght of the two sides of the BoundingBox divided by two
    float b = (this->max_y - this->min_y);

    return a*b;
}

// Scale functions
void BoundingBox::resize(float scale) {
    min_x = min_x * scale;
    min_y = min_y * scale;
    max_x = max_x * scale;
    max_y = max_y * scale;
}

// Segment colliding function
bool BoundingBox::isSegmentColliding(const Segment &seg) const{
    float xp1 = seg.getP1().x;
    float yp1 = seg.getP1().y;
    float xp2 = seg.getP2().x;
    float yp2 = seg.getP2().y;

    float x_seg_max = 0;
    float x_seg_min = 0;
    float y_seg_max = 0;
    float y_seg_min = 0;

    if(xp1 >= xp2) {
        x_seg_max = xp1;
        x_seg_min = xp2;
    } else {
        x_seg_max = xp2;
        x_seg_min = xp1;
    }

    if(yp1 >= yp2) {
        y_seg_max = yp1;
        y_seg_min = yp2;
    } else {
        y_seg_max = yp2;
        y_seg_min = yp1;
    }

    if(x_seg_max < min_x) {
        return false;
    } else if (x_seg_min > max_x) {
        return false;
    } else if(y_seg_max < min_y) {
        return false;
    } else if(y_seg_min > max_y) {
        return false;
    } else {
        return true;
    }

}

// Arc collision function
bool BoundingBox::isArcColliding(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const {
    float a = (this->max_x - this->min_x)/2; // the lenght of the two sides of the BoundingBox divided by two
    float b = (this->max_y - this->min_y)/2;
    Segment s = Segment(this->center,arc_center);
    float s_length = s.getLength();

    float polygon_radius = sqrt( a*a + b*b ); // radius of the circle where inside there is this BoundingBox

    if( (polygon_radius + radius) < s_length) {
        return false;
    }

    float xc = arc_center.x;
    float yc = arc_center.y;

    float theta_start = atan2((start.y - yc),(start.x - xc));
    float theta_finish = atan2((finish.y - yc),(finish.x - xc));
    float thetat = atan2( this->center.y - arc_center.y, this->center.x - arc_center.x);
    thetat = Utility::mod2pi(thetat);
    theta_start = Utility::mod2pi(theta_start);
    theta_finish = Utility::mod2pi(theta_finish);


    if(clockwise) {
            if(theta_finish >= theta_start && thetat >= theta_start && thetat <= theta_finish) {
                return true;
            } else if(theta_finish < theta_start && ( ( theta_start <= thetat) || (thetat <= theta_finish) ) ) {
                return true;
            } else {
                //cout << "Bounding Box has no intersection " << endl;
            }
        } else {
            if(theta_finish >= theta_start && (thetat <= theta_start || thetat >= theta_finish) ) {
                return true;
            } else if( theta_start > theta_finish && theta_finish <= thetat && thetat <= theta_start ) {
                return true;
            } else {
                //cout << "Bounding Box has no intersection " << endl;
            }
        }

    // the arc in not touching the boundingbox, but maybe it is inside if the start point (or the finish point, is the same) is inside the bounding box
    return this->isPointInside(start);
}

// DubinsArc collision function
bool BoundingBox::isDubinsArcColliding(const DubinsArc &da) const {

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
        bool clockwise;
        if( da.getK() < 0) {
            float xc = cos( da.getTh0() - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( da.getTh0() - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = Point(xc + da.getX0(),yc + da.getY0());
            clockwise = false;
        } else {
            float xc = cos( da.getTh0() + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( da.getTh0() + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = Point(xc + da.getX0(),yc + da.getY0());
            clockwise = true;
        }

        return this->isArcColliding(center,radius,p1,p2,clockwise);

    }


}

// Dubins Cruve colliding function
bool BoundingBox::isDubinsCurveColliding(const DubinsCurve &dc) const{
    for(unsigned int i = 0; i < 3; i++) {
        DubinsArc da = dc.getArc(i);
        if(this->isDubinsArcColliding(da)) {
            return true;
        }
    }
    return false;
}

// Point inside the BoundingBox function
bool BoundingBox::isPointInside(const Point2f &p) const {
    if( p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y) {
        return true;
    } else {
        return false;
    }

}

// Get min x
float BoundingBox::getMinX() const {
    return min_x;
}

// Get min y
float BoundingBox::getMinY() const {
    return min_y;
}

// Get max x
float BoundingBox::getMaxX() const {
    return max_x;
}

// Get max y
float BoundingBox::getMaxY() const {
    return max_y;
}


bool BoundingBox::isDubinsArcTouching(const DubinsArc &da) const {
    float k = da.getK();
    Segment s1 = Segment(Point2f(min_x,min_y),Point2f(min_x,max_y));
    Segment s2 = Segment(Point2f(min_x,min_y),Point2f(max_x,min_y));
    Segment s3 = Segment(Point2f(max_x,max_y),Point2f(min_x,max_y));
    Segment s4 = Segment(Point2f(max_x,max_y),Point2f(max_x,min_y));

    // straight line case
    if(k == 0) {
        Point2f p1 = Point2f(da.getX0(), da.getY0() ) ;
        Point2f p2 = Point2f(da.getXf(), da.getYf() ) ;
        Segment dubinsLine = Segment(p1,p2);

        return (s1.isCollidingWithSegment(dubinsLine) || s2.isCollidingWithSegment(dubinsLine) || s3.isCollidingWithSegment(dubinsLine) || s4.isCollidingWithSegment(dubinsLine) );

    // arc case
    } else {
        float radius = abs(1 / k);
        Point2f p1 = Point2f(da.getX0(), da.getY0() ) ;
        Point2f p2 = Point2f(da.getXf(), da.getYf() ) ;
        Point2f center;
        bool clockwise;
        if( da.getK() < 0) {
            float xc = cos( da.getTh0() - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( da.getTh0() - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = Point(xc + da.getX0(),yc + da.getY0());
            clockwise = false;
        } else {
            float xc = cos( da.getTh0() + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( da.getTh0() + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = Point(xc + da.getX0(),yc + da.getY0());
            clockwise = true;
        }

        return ( s1.isCollidingWithArc(center,radius,p1,p2,clockwise) || s2.isCollidingWithArc(center,radius,p1,p2,clockwise)
        || s3.isCollidingWithArc(center,radius,p1,p2,clockwise) || s4.isCollidingWithArc(center,radius,p1,p2,clockwise) )  ;

    }
}

// get the segments of created by the points of the bounding box
vector<Segment> BoundingBox::getSegments() {
    Point2f p1 = Point(min_x,min_y);
    Point2f p2 = Point(max_x,min_y);
    Point2f p3 = Point(min_x,max_y);
    Point2f p4 = Point(max_x,max_y);

    Segment s1 = Segment(p1,p2);
    Segment s2 = Segment(p1,p3);
    Segment s3 = Segment(p4,p2);
    Segment s4 = Segment(p4,p3);

    vector<Segment> segments = {};
    segments.push_back(s1);
    segments.push_back(s2);
    segments.push_back(s3);
    segments.push_back(s4);

    return segments;

}

// TODO
BoundingBox& BoundingBox::getBoundingBox(){
  return *this;
}

bool BoundingBox::bbIntersectsDubinsCurve(DubinsCurve &dc){
  std::cout << "BB bbIntersectsDubinsCurve" << std::endl;
  return this->isDubinsCurveColliding(dc);
}
