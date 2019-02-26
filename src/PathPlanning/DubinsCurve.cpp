#include "DubinsCurve.hpp"
#include<cstdlib>

// Empty constructor
DubinsCurve::DubinsCurve() : Shape(){
    this->arc1 = DubinsArc();
    this->arc2 = DubinsArc();
    this->arc3 = DubinsArc();
}

// Full constructor
DubinsCurve::DubinsCurve(double x0,double y0, double th0, double l1, double l2, double l3, double k1, double k2, double k3) : Shape(){
    this->arc1 = DubinsArc(x0,y0,th0,k1,l1);
    double x1= arc1.getXf();
    double y1 = arc1.getYf();
    double th1 = arc1.getThf();

    this->arc2 = DubinsArc(x1,y1,th1,k2,l2);
    double x2= arc2.getXf();
    double y2 = arc2.getYf();
    double th2 = arc2.getThf();

    this->arc3 = DubinsArc(x2,y2,th2,k3,l3);

    Point2f center = arc2.getCenter();
    this->setCenter(center);
}

// Get arc method
const DubinsArc& DubinsCurve::getArc(int pos) const{
  switch(pos){
    case 0: return arc1;
    case 1: return arc2;
    case 2: return arc3;
    default: return arc1;
  }

}

// Get total lenght method
double DubinsCurve::getTotalLenght() const{
    return this->arc1.getL() + this->arc2.getL() + this->arc3.getL();
}

// add points to the curve
double DubinsCurve::addPoints(double overlength, Path &path) const{
  overlength = arc1.addPoints(overlength, path);
  overlength = arc2.addPoints(overlength, path);
  return arc3.addPoints(overlength, path);
}

// Get the initial x of the first arc
double DubinsCurve::getX0() const {
    return this->arc1.getX0();
}

// Get the initial y of the first arc
double DubinsCurve::getY0() const {
    return this->arc1.getY0();
}

// Get the initial angle of the first arc
double DubinsCurve::getTh0() const {
    return this->arc1.getTh0();
}

// Get the initial x of the second arc
double DubinsCurve::getX1() const{
    return this->arc1.getXf();
}

// Get the initial y of the second arc
double DubinsCurve::getY1() const{
    return this->arc1.getYf();
}

// Get the initial angle of the second arc
double DubinsCurve::getTh1() const{
    return this->arc1.getThf();
}

// Get the initial x of the third arc
double DubinsCurve::getX2() const{
    return this->arc2.getXf();
}

// Get the initial y of the third arc
double DubinsCurve::getY2() const{
    return this->arc2.getYf();
}

// Get the initial angle of the third arc
double DubinsCurve::getTh2() const{
    return this->arc2.getThf();
}

// Get the final x of the third arc
double DubinsCurve::getXf() const{
    return this->arc3.getXf();
}

// Get the final y of the third arc
double DubinsCurve::getYf() const{
    return this->arc3.getYf();
}

// Get the final angle of the third arc
double DubinsCurve::getThf() const{
    return this->arc3.getThf();
}

// < operator to confront the leght
bool DubinsCurve::operator < (const DubinsCurve& other) const {
    return this->getTotalLenght() < other.getTotalLenght();
}

// Print function
void DubinsCurve::print() const {
    cout << "Dubins Cruve composed by: " << endl;
    arc1.print();
    arc2.print();
    arc3.print();

}
