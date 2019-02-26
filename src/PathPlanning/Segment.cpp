#include "Segment.hpp"

// Empty constructor
Segment::Segment() : Shape(Point2f(0.5,0.5)) {
    this->p1 = Point(0,0);
    this->p2 = Point(1,1);
}

// Full constructor
Segment::Segment(Point2f p1,Point2f p2) : Shape(Point2f (p1.x/2+p2.x/2,p1.y/2+p2.y/2)) {
    if(p1.x == p2.x && p1.y == p2.y) {
        this->p1 = Point2f(0,0);
        this->p2 = Point2f(1,1);
    } else {
        this->p1 = p1;
        this->p2 = p2;
    }

}

// Get P1 method
Point2f Segment::getP1() const{
    return this->p1;
}

// Get P2 method
Point2f Segment::getP2() const{
    return this->p2;
}

// Set P1 method
void Segment::setP1(Point2f p1) {
    this->p1 = p1;
}

// Set P2 method
void Segment::setP2(Point2f p2) {
    this->p1 = p2;
}

// print method
void Segment::print() const {
    cout << "[ P1=(";
    printf("%4.4f",p1.x);
    cout << ",";
    printf("%4.4f",p1.y);
    cout << ") , P2=(";
    printf("%4.4f",p2.x);
    cout << ",";
    printf("%4.4f",p2.y);
    cout << ") ]" << endl;
}

// Calculate the lenght of the segment
float Segment::getLength() const {
    return sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) );
}

// Return the maximum value of x
float Segment::getMaxX() const{
    if(p1.x > p2.x) {
        return p1.x;
    } else {
        return p2.x;
    }
}

// Return the maximum value of y
float Segment::getMaxY()  const {
    if(p1.x > p2.y) {
        return p1.y;
    } else {
        return p2.y;
    }
}

// Return the minimum value of x
float Segment::getMinX() const {
    if(p1.x < p2.x) {
        return p1.x;
    } else {
        return p2.x;
    }
}

// Return the minimum value of y
float Segment::getMinY() const {
    if(p1.x < p2.y) {
        return p1.y;
    } else {
        return p2.y;
    }
}

// Segment collision method
bool Segment::isCollidingWithSegment(const Segment &seg) const {
    // get this segment coordinates
    float x1 = p1.x;
    float x2 = p2.x;
    float y1 = p1.y;
    float y2 = p2.y;
    // get the other segment coordinates
    Point2f p3 = seg.getP1();
    Point2f p4 = seg.getP2();
    float x3 = p3.x;
    float x4 = p4.x;
    float y3 = p3.y;
    float y4 = p4.y;

    // calculate the determinant of the matrix that is the result of equating the equations of the two segments
    float determinant = (x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3);

    // the system have some solutions if det != 0
    if(determinant != 0) {
        // determine t and u of the intersection, that with the formula p1 + t(p2-p1) with t->[0,1] and p3 + u(p4-p3) with u->[0,1] they represent the segments
        float t = ( (y3 - y4)*(x1 - x3) +(x4 - x3)*(y1 - y3)  ) / determinant;
        float u = ( (y1 - y2)*(x1 - x3) +(x2 - x1)*(y1 - y3)  ) / determinant;

        // if both t an u are in the [0,1] interval the solutions are in the two segments
        if(t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            //cout << "This segment " << p1 << "," << p2 << " touch segment " << seg.getP1() << "," << seg.getP2() << "in t " << t << " and u " << u << endl;
            return true;
        } else { // otherwise the two lines have an intersection, but it is not in the segments
            //cout << "Segment do not touches the input segment" << endl;
            return false;
        }
        // the system has no solutions, therefore the lines are parallel and have no point in common or infinite points in common
    } else {
        // the two segments are parallel, and therefore they have the same coefficients
        // check if the two lines intersect the x=0 line in the same spot (q1 and q2 are the y-coordinates of the lines intersecting the x=0 line)
        if(x2 - x1 != 0 && x4 != x3){
            float q1 = y1 - ( x1 / (x2-x1) ) * (y2 - y1);
            float q2 = y3 - ( x3 / (x4-x3) ) * (y4 - y3);

            // the two segments are on the same line if q1 = q2
            if( (q1-q2)*(q1-q2) < 0.00001) {

                // calculate the t value of the second segment points
                float t3 = (x3 - x1);
                t3 = t3 / float(x2 - x1);
                float t4 = (x4 - x1);
                t4 = t4 / float(x2 - x1);

                // calculate the u value of the first segment points
                float u1 = (x1 - x3);
                u1 = u1 / float(x4 - x3);
                float u2 = (x2 - x3);
                u2 = u2 / float(x4 - x3);

                // if t3 ot t4 are int the interval [0,1] it means that the ends of segment two lies in segment one,
                // if u1 or u2 are in the interval [0,1] it means that the ends of segment one lies on segment two
                if( ( t3 >= 0 && t3 <= 1 ) || (t4 >= 0 && t4 <= 1) || (u1 >= 0 && u1 <= 1) || (u2 >= 0 && u2 <= 1) ){
                    //cout << "This segment " << p1 << "," << p2 << " touch segment " << seg.getP1() << "," << seg.getP2() << " and they are parallel on the same line" << endl;
                    return true;
                } else {
                    //cout << "Segment  is parallel to the input segment, on the same line and it do not touches it" << endl;
                    return false;
                }
            // the segments are not on the same line if q1 != q2
            } else {
                //cout << "Segment  is parallel to the input segment but not on the same line" << endl;
                return false;
            }
        } else { // these segments are vertical
            if(x1 != x3) {
                return false; // vertical segments but parallel
            } else {    // vertial segments on the same line, check the y value
                if(this->getMaxY() < seg.getMinY() ) {
                    return false;
                } else if(this->getMinY() > seg.getMaxY()) {
                    return false;
                } else {
                    //cout << "This segment " << p1 << "," << p2 << " touch segment " << seg.getP1() << "," << seg.getP2() << " and they are parallel on the same line" << endl;
                    return true;
                }
            }

        }
    }
    return false;
}

// Arc collision method
bool Segment::isCollidingWithArc(const Point2f &arc_center, float radius, const Point2f &start,const Point2f &finish, bool clockwise) const {
    // get the center of the circle where the line lies
    float xc = arc_center.x;
    float yc = arc_center.y;

    // get the segment coordinates
    float x1 = this->getP1().x;
    float x2 = this->getP2().x;
    float y1 = this->getP1().y;
    float y2 = this->getP2().y;

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
            t_vector.push_back(1-t1);
        }

    } else { // two solutions
        float t1 = (-c2 + sqrt( delta ) ) / (2*c1);
        float t2 = (-c2 - sqrt( delta) ) / (2*c1);
        if(t1 >= 0 && t1 <= 1) { // if t1 is in the t[0,1] interval the solution is in the segment
            t_vector.push_back(1-t1);
        }
        if(t2 >= 0 && t2 <= 1) { // if t2 is in the t[0,1] interval the solution is in the segment
            t_vector.push_back(1-t2);
        }
        //cout << "t1: " << t1 << " t2: " << t2 << endl;
    }

    // for each valid solution found compute if there are collisions
    for(unsigned int j = 0; j < t_vector.size(); j++) {
        // calculate the intersection coordinates
        float x_intersection = x1 + t_vector[j]*(x2 - x1);
        float y_intersection = y1 + t_vector[j]*(y2 - y1);

        // calculate angles with the formula atan( (y-yc)*radius, (x-xc)*radius )
        float thetat = atan2((y_intersection - yc),(x_intersection - xc));
        float theta_start = atan2((start.y - yc),(start.x - xc));
        float theta_finish = atan2((finish.y - yc),(finish.x - xc));
        thetat = Utility::mod2pi(thetat);
        theta_start = Utility::mod2pi(theta_start);
        theta_finish = Utility::mod2pi(theta_finish);
        //cout << "thetat " << thetat << " t_start : " << theta_start << " th_f: " << theta_finish << endl;

        if(clockwise) {
            if(theta_finish >= theta_start && thetat >= theta_start && thetat <= theta_finish) {
                return true;
            } else if(theta_finish < theta_start && ( ( theta_start <= thetat) || (thetat <= theta_finish) ) ) {
                return true;
            } else {
                //cout << "Segment has intersection with the circle in" << t_vector[j] << " t-coordinate but it does not touch the segment" << endl;
            }
        } else {
            if(theta_finish >= theta_start && (thetat <= theta_start || thetat >= theta_finish) ) {
                return true;
            } else if( theta_start > theta_finish && theta_finish <= thetat && thetat <= theta_start ) {
                return true;
            } else {
                //cout << "Segment has intersection with the circle in" << t_vector[j] << " t-coordinate but it does not touch the segment" << endl;
            }
        }

    }
    //cout << "The segment has no intersection with the segment, but some with the circle" << endl;
    return false;
}

// Get the m parameter in the equation of the line created by the two points
double Segment::getMLine() const {
    if( p1.x != p2.x)  {
        return (p1.y - p2.y) / (p1.x - p2.x); // formula m = (y1-y2)/(x1-x2)
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    } 
}

// Get the q parameter in the equation of the line created by the two points
double Segment::getQLine() const {
    if( p1.x != p2.x)  {
        return (p1.x*p2.y - p2.x*p1.y)/( p1.x - p2.x ) ; // formula q = (x1y2 - x2y1) / (x1-x2)
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}


Point2f Segment::getCollisionPointWithSegment(const Segment &seg) const {
    // get this segment coordinates
    float x1 = p1.x;
    float x2 = p2.x;
    float y1 = p1.y;
    float y2 = p2.y;
    // get the other segment coordinates
    Point2f p3 = seg.getP1();
    Point2f p4 = seg.getP2();
    float x3 = p3.x;
    float x4 = p4.x;
    float y3 = p3.y;
    float y4 = p4.y;

    // calculate the determinant of the matrix that is the result of equating the equations of the two segments
    float determinant = (x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3);

    // the system have some solutions if det != 0
    if(determinant != 0) {
        // determine t and u of the intersection, that with the formula p1 + t(p2-p1) with t->[0,1] and p3 + u(p4-p3) with u->[0,1] they represent the segments
        float t = ( (y3 - y4)*(x1 - x3) +(x4 - x3)*(y1 - y3)  ) / determinant;
        float u = ( (y1 - y2)*(x1 - x3) +(x2 - x1)*(y1 - y3)  ) / determinant;

        // if both t an u are in the [0,1] interval the solutions are in the two segments
        if(t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            //cout << "This segment " << p1 << "," << p2 << " touch segment " << seg.getP1() << "," << seg.getP2() << "in t " << t << " and u " << u << endl;
            return Point2f(x1+t*(x2-x1),y1+t*(y2-y1));
        } else { // otherwise the two lines have an intersection, but it is not in the segments
            //cout << "Segment do not touches the input segment" << endl;
            return Point2f(0,0);
        }
        // the system has no solutions, therefore the lines are parallel and have no point in common or infinite points in common
    } else {
        // the two segments are parallel, and therefore they have the same coefficients
        // check if the two lines intersect the x=0 line in the same spot (q1 and q2 are the y-coordinates of the lines intersecting the x=0 line)
        if(x2 - x1 != 0 && x4 != x3){
            float q1 = y1 - ( x1 / (x2-x1) ) * (y2 - y1);
            float q2 = y3 - ( x3 / (x4-x3) ) * (y4 - y3);

            // the two segments are on the same line if q1 = q2
            if( (q1-q2)*(q1-q2) < 0.00001) {

                // calculate the t value of the second segment points
                float t3 = (x3 - x1);
                t3 = t3 / float(x2 - x1);
                float t4 = (x4 - x1);
                t4 = t4 / float(x2 - x1);

                // calculate the u value of the first segment points
                float u1 = (x1 - x3);
                u1 = u1 / float(x4 - x3);
                float u2 = (x2 - x3);
                u2 = u2 / float(x4 - x3);

                // if t3 ot t4 are int the interval [0,1] it means that the ends of segment two lies in segment one,
                // if u1 or u2 are in the interval [0,1] it means that the ends of segment one lies on segment two
                if( t3 >= 0 && t3 <= 1 ) {
                    Point2f(x1+t3*(x2-x1),y1+t3*(y2-y1));
                } else if( t4 >= 0 && t4 <= 1 ){
                    Point2f(x1+t4*(x2-x1),y1+t4*(y2-y1));
                } else if(u1 >= 0 && u1 <= 1) {
                    Point2f(x3+u1*(x4-x3),y3+u1*(y4-y3));

                } else if (u2 >= 0 && u2 <= 1){
                    Point2f(x3+u2*(x4-x3),y3+u2*(y4-y3));
                } else {
                    //cout << "Segment  is parallel to the input segment, on the same line and it do not touches it" << endl;
                    return Point2f(0,0);
                }
            // the segments are not on the same line if q1 != q2
            } else {
                //cout << "Segment  is parallel to the input segment but not on the same line" << endl;
                return Point2f(0,0);
            }
        } else { // these segments are vertical
            if(x1 != x3) {
                return Point2f(0,0); // vertical segments but parallel
            } else {    // vertial segments on the same line, check the y value
                if(this->getMaxY() < seg.getMinY() ) {
                    return Point2f(0,0);
                } else if(this->getMinY() > seg.getMaxY()) {
                    return Point2f(0,0);
                } else {
                    //cout << "This segment " << p1 << "," << p2 << " touch segment " << seg.getP1() << "," << seg.getP2() << " and they are parallel on the same line" << endl;
                    return Point2f(0,0);
                }
            }

        }
    }
    return Point2f(0,0);
}

// Comparison operator
bool Segment::operator ==(const Segment &other) const {

    if( (abs(p1.x - other.getP1().x) < 0.00001 && abs(p1.y - other.getP1().y) < 0.00001 && abs(p2.x - other.getP2().x) < 0.00001 && abs(p2.y - other.getP2().y) < 0.00001)
    || (abs(p1.x - other.getP2().x) < 0.00001 && abs(p1.y - other.getP2().y) < 0.00001 && abs(p2.x - other.getP1().x) < 0.00001 && abs(p2.y - other.getP1().y) < 0.00001) ) {
        return true;
    } else {
        return false;
    }

}
