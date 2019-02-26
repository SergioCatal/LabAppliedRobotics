#include "Obstacle.hpp"

// Empty constructor
Obstacle::Obstacle() :Polygon() {
    calculateBoundingBox();
}

// Full constructor
Obstacle::Obstacle(Point2f center, vector<Point2f> vertices) : Polygon(center,vertices) {
    calculateBoundingBox();
}

// Function to get the smallest and biggest x and y coordinates of the polygon and it uses it to generate the bounding box object
void Obstacle::calculateBoundingBox() {
    float min_x;
    float min_y;
    float max_x;
    float max_y;

    if(vertices.size() != 0) {
        min_x = vertices[0].x;
        min_y = vertices[0].y;
        max_x = vertices[0].x;
        max_y = vertices[0].y;
    } else {
        this->boundingBox = BoundingBox();
        return;
    }

    for(unsigned int i = 0; i < this->vertices.size();i++) {
        float current_x = vertices[i].x;
        float current_y = vertices[i].y;

        if(current_x > max_x) {
            max_x = current_x;
        }
        if(current_x < min_x) {
            min_x = current_x;
        }
        if(current_y > max_y) {
            max_y = current_y;
        }
        if(current_y < min_y) {
            min_y = current_y;
        }
    }

    this->boundingBox = BoundingBox(min_x,min_y,max_x,max_y);
}

// Edge clipping function
void Obstacle::clipperEdges(float offset) {

    this->Polygon::clipperEdges(offset);
    this->calculateBoundingBox();

}

// Resize function
void Obstacle::resize(float scale) {

    this->Polygon::resize(scale);
    calculateBoundingBox();

}

// Arc colliding with bouding box
bool Obstacle::isArcCollidingWithBoundingBox(const Point2f &center, float radius, const Point2f &start, const Point2f &finish,bool clockwise) const {
    return this->boundingBox.isArcColliding(center,radius,start,finish,clockwise);
}

// Arc colliding with bouding box
bool Obstacle::isSegmentCollidingWithBoundingBox(const Point2f &p1, const Point2f &p2) const {
    return this->boundingBox.isSegmentColliding(Segment(p1,p2));
}

// Dubins arc colliding with bounding box
bool Obstacle::isDubinsArcCollidingWithBoundingBox(const DubinsArc &da) const {
    float k = da.getK();

    // straight line case
    if(k == 0) {
        Point2f p1 = Point2f(da.getX0(), da.getY0() ) ;
        Point2f p2 = Point2f(da.getXf(), da.getYf() ) ;

        return this->boundingBox.isSegmentColliding(Segment(p1,p2));;
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

        return this->boundingBox.isArcColliding(center,radius,p1,p2,clockwise);

    }
}

// Dubins arc colliding with bounding box
bool Obstacle::isDubinsCurveCollidingWithBoundingBox(const DubinsCurve &dc) const {
    for(unsigned int i = 0; i < 3; i++) {
        DubinsArc da = dc.getArc(i);
        if(this->isDubinsArcCollidingWithBoundingBox(da)) {
            return true;
        }
    }
    return false;
}

// Print method
void Obstacle::print() const {
    cout << "Obstacle  ";
    this->Polygon::print();
    cout << "With bounding box: ";
    this->boundingBox.print();
}

// Get the bounding box method
BoundingBox& Obstacle::getBoundingBox() {
    return boundingBox;
}

bool Obstacle::bbIntersectsDubinsCurve(DubinsCurve &dc){
  return isDubinsCurveColliding(dc);
}

// Optimize is poin inside with a bounding box
bool Obstacle::isPointInside(const Point2f &p) const {
    if(this->boundingBox.isPointInside(p)) {
        return Polygon::isPointInside(p);
    } else {
        return false;
    }
}
