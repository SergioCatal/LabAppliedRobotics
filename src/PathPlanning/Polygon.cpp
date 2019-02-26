#include "Polygon.hpp"
using namespace ClipperLib;

// public constructor
Polygon::Polygon() : FilledConvexShape(){
    this->vertices = {};
    this->segments = {};
}

// Full constructor
Polygon::Polygon(Point2f center, vector<Point2f> vertices) : FilledConvexShape(center) {
    this->vertices = vertices;
    calculateSegments();
}

// get the number of vertices
int Polygon::getNVertices() {
    return vertices.size();
}

// Method that return the vertices of the polygon
vector<Point2f> Polygon::getVertices() const { 
    return this->vertices;
}

// Method that return the area of the polygon
float Polygon::getArea() const {
    return contourArea(vertices);
}

vector<Segment> Polygon::getSegments() const {
    return this->segments;
}

// Simple print method
void Polygon::print() const {
    cout << "Polygon center: " << center << " ,N vertices: " << vertices.size() << " ,vertices list: " << endl;
    for(unsigned int i = 0; i < vertices.size();i++) {
        cout << vertices[i] << " ";
    }
    cout << endl << "N segments " << segments.size() << " : " << endl;
    for(unsigned int i = 0; i < segments.size(); i++) {
        this->segments[i].print();
    }
    cout << endl;
}

// Copy constructor override to avoid problems
Polygon::Polygon(const Polygon& other) : FilledConvexShape(other.getCenter()){
    this->center = other.getCenter();
    this->vertices = {};
    this->segments = {};
    center = other.getCenter();
    for(unsigned int i = 0; i < other.getVertices().size();i++) {
        Point2f p = other.getVertices()[i];
        vertices.push_back(p);
    }
    calculateSegments();
}

// Assignment operator override to avoid problems
Polygon& Polygon::operator= (const Polygon& other){
    this->center = other.getCenter();
    this->vertices = {};
    this->segments = {};
    center = other.getCenter();
    for(unsigned int i = 0; i < other.getVertices().size();i++) {
        Point2f p = other.getVertices()[i];
        this->vertices.push_back(p);
    }
    calculateSegments();
    return *this;
}

// Divide the polygon in a list of segments
void Polygon::calculateSegments() {
    this->segments = {};

    if(this->vertices.size() > 0) {

        for(unsigned int i = 0; i < this->vertices.size() - 1; i++) {
        Segment s = Segment( vertices[i], vertices[i+1]);
        this->segments.push_back(s);
        }

        Segment s = Segment(vertices[vertices.size() - 1], vertices[0]);
        this->segments.push_back(s);
    }

}

// edge clipping function
void Polygon::clipperEdges(float offset) {

    const double INT_ROUND = 1000.;
    const double OFFSET = offset * INT_ROUND;

    ClipperLib::Path srcPoly;
    ClipperLib::Paths newPoly;

    for(unsigned int i = 0; i < this->vertices.size(); i++) {
        int x = this->vertices[i].x * INT_ROUND;
        int y = this->vertices[i].y * INT_ROUND;
        srcPoly << ClipperLib::IntPoint(x,y);
    }

    ClipperLib::ClipperOffset co;
    if(this->vertices.size() == 3) {
        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        //co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedLine);
    } else {
        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        //co.AddPath(srcPoly, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    }

    co.Execute(newPoly, OFFSET);

    this->vertices.clear();

    for(const ClipperLib::Path &path: newPoly) {

        for(const ClipperLib::IntPoint &pt: path) {
            float x = pt.X / INT_ROUND;
            float y = pt.Y / INT_ROUND;
            this->vertices.push_back( Point2f(x,y));
        }

    }

    this->segments.clear();
    this->calculateSegments();

}

// resize every point of the polygon
void Polygon::resize(float scale) {

    center.x = center.x * scale;
    center.y = center.y * scale;

    for(unsigned int i = 0; i < vertices.size(); i++) {
        vertices[i].x = vertices[i].x * scale;
        vertices[i].y = vertices[i].y * scale;
    }

    calculateSegments();

}

// function that checks if a segment is touching the polygon
bool Polygon::isSegmentColliding(const Segment &seg) const {
    Point2f p1 = seg.getP1();
    Point2f p2 = seg.getP2();

    for(unsigned int i = 0; i < segments.size(); i++) {
        if(segments[i].isCollidingWithSegment(Segment(p1,p2))) {
            return true;
        }
    }

    // no collison between the segment and the segments of the polygon, but the segment can be inside if the p1 (or p2) is inside the polygon
    return (this->isPointInside(seg.getP1()) || this->isPointInside(seg.getP2()));
}

// function that compute if there is a collision between the polygon and an arc
bool Polygon::isArcColliding(const Point2f &arc_center, float radius, const Point2f &start, const Point2f &finish, bool clockwise) const {

    for(unsigned int i = 0; i < segments.size(); i++) {
        if(segments[i].isCollidingWithArc(arc_center,radius,start,finish,clockwise)) {

            return true;
        }
    }
    // no collison between arc and the segments of the polygon, but the arc can be inside if the start (or the finish) is inside the polygon
    return (this->isPointInside(start) || this->isPointInside(finish));
}

// function that cheks if a dubins arc is colliding with the polygon
bool Polygon::isDubinsArcColliding(const DubinsArc &da) const {

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

// if a dubins curve is touching
bool Polygon::isDubinsCurveColliding(const DubinsCurve &dc) const {
    for(unsigned int i = 0; i < 3; i++) {
        DubinsArc da = dc.getArc(i);
        if(this->isDubinsArcColliding(da)) {
            return true;
        }
    }
    return false;
}

// return true if a point ins inside
bool Polygon::isPointInside(const Point2f &p) const {

    for(unsigned int i = 0; i < segments.size(); i++) {
        Point2f confront; // other vertice of the polygon different from the two points forming the current segment, in order to understand the
                        // position of the polygon in respect to the line

        if(i >= 1) {
            confront = segments[i-1].getP1();
        } else {
            confront = segments[i+1].getP2();
        }

        double m = segments[i].getMLine();  // get the m coefficient of the line created by the two points of the segment

        if(isnan(m)) {  // case vertical line

            if(confront.x > segments[i].getP1().x) { // the polygon is on the right of the vertical segment
                if(p.x > segments[i].getP1().x) {
                    continue;
                } else {
                    return false;
                }
            } else {    // the polygon is on the left of the vertical line
                if(p.x < segments[i].getP1().x) {
                    continue;
                } else {
                    return false;
                }
            }

        }
        double q = segments[i].getQLine();

        if( (confront.y - m*confront.x) >= q) { // the points in the polygon satisfy the equation y - m*x > q

            if( (p.y - m*p.x) >= q) {
                continue;
            } else {
                return false; 
            }

        } else {// the points in the polygon satisfy the equation y - m*x < q

            if( (p.y - m*p.x) <= q) {
                continue;
            } else {
                return false;
            }

        }

    }
    return true; // if the point satisfy all the disequations of the segments is inside the polygon

}
