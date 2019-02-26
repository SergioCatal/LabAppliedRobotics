#include "DubinsArc.hpp"
#include "Utility.hpp"
#include <cmath>
#include <iostream>

// Empty constructor
DubinsArc::DubinsArc() : Shape() {
	this->x0 = 0;
	this->y0 = 0;
	this->th0 = 0;
	this->k = 0;
	this->l = 0;
	this->xf = 0;
	this->yf = 0;
	this->thf = 0;
}

// Full constructor
DubinsArc::DubinsArc(double x0, double y0, double th0, double k, double l, double xf, double yf, double thf) : Shape() {
	this->x0 = x0;
	this->y0 = y0;
	this->th0 = th0;
	this->k = k;
	this->l = l;
	this->xf = xf;
	this->yf = yf;
	this->thf = thf;

    // straight line case for the calculation of the center
    if(k == 0) {
        float xc = x0 + cos(th0)*l/2;
		float yc = y0 + sin(th0)*l/2;
		this->setCenter(Point2f(xc,yc)); // set the center of the arc as the medium point of the segment
    // arc case for the calculation of the center
    } else {
        float radius = 1 / k;
        Point2f center;
        if( k < 0) {
            float xc = cos( th0 - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( th0 - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = Point(xc + x0,yc + y0);
        } else {
            float xc = cos( th0 + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( th0 + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = Point(xc + x0,yc + y0);
        }

        this->setCenter(center);

    }

}

// Constructor with only the initial parameters
DubinsArc::DubinsArc(double x0, double y0, double th0, double k, double l) {
	this->x0 = x0;
	this->y0 = y0;
	this->th0 = th0;
	this->k = k;
	this->l = l;
	this->xf = this->x0 + this->l * Utility::sinc(this->k *  this->l / 2.0) * std::cos(th0 + k * this->l / 2);
	this->yf = this->y0 + this->l * Utility::sinc(this->k *  this->l / 2.0) * std::sin(th0 + k * this->l / 2);
	this->thf = Utility::mod2pi(this->th0 + this->k * this->l);

	// straight line case for the calculation of the center
    if(k == 0) {
        float xc = x0 + cos(th0)*l/2;
		float yc = y0 + sin(th0)*l/2;
		this->setCenter(Point2f(xc,yc)); // set the center of the arc as the medium point of the segment
    // arc case for the calculation of the center
    } else {
        float radius = 1 / k;
        Point2f center;
        if( k < 0) {
            float xc = cos( th0 - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( th0 - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = Point(xc + x0,yc + y0);
        } else {
            float xc = cos( th0 + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( th0 + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = Point(xc + x0,yc + y0);
        }

        this->setCenter(center);
    }
}

// TODO better
double sampling_length = 10;

// add point method
double DubinsArc::addPoints(double overlength, Path &path) const{
	Pose pose;
	double s0 = 0;
	double arc_s = sampling_length;

	pose.kappa = k;
	if(path.points.size() == 0){
		pose.s = 0;
		pose.x = x0;
		pose.y = y0;
		pose.theta = th0;
		path.points.push_back(pose);
		overlength = 0;
	} else {
		s0 = path.points.back().s + overlength;
		arc_s = sampling_length - overlength;
	}
	while(arc_s <= l){
		pose.s = s0 + arc_s;
		getPoint(arc_s, pose);
		Pose &p = path.points.back();
		if(std::sqrt(std::abs(p.x - pose.x) * std::abs(p.x - pose.x) + std::abs(p.y - pose.y) * std::abs(p.y - pose.y)) > 20.01){
			std::cout << "INCONSISTENCY:";
			this->print();
			std::cout << "INDEX of NEW POINT: " << path.points.size() << "ENDING LENGTH: " << s0 + l << std::endl;
		}
		path.points.push_back(pose);
		arc_s += sampling_length;
	}

	return  sampling_length + l - arc_s;
}

// get a point method
void DubinsArc::getPoint(double s, Pose& pose) const{
	pose.theta = s*k + th0;
	if(k == 0){
		pose.x = x0 + (xf-x0)/l * s;
		pose.y = y0 + (yf-y0)/l * s;
	} else {
		pose.x = x0 + (std::sin(pose.theta) - std::sin(th0))/k;
		pose.y = y0 + (std::cos(th0) - std::cos(pose.theta))/k;
	}
}

// from a dubinc arc get the information to create the arc
void DubinsArc::getArc(Point2f &center, float &radius, Point2f &start_point, Point2f &end_point, bool &clockwise) {

    float k = this->getK();

    // straight line case
    if(k == 0) {
        Point2f p1 = Point2f(this->getX0(), this->getY0() ) ;
        Point2f p2 = Point2f(this->getXf(), this->getYf() ) ;
        Segment seg = Segment(p1,p2);
        center = seg.getCenter();
        radius = 0;
        start_point = p1;
        end_point = p2;
        clockwise = true;

    // arc case
    } else {
        radius = abs(1 / k);
        Point2f p1 = Point2f(this->getX0(),this->getY0() ) ;
        Point2f p2 = Point2f(this->getXf(), this->getYf() ) ;
        if( this->getK() < 0) {
            float xc = cos( this->getTh0() - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( this->getTh0() - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = Point(xc + this->getX0(),yc + this->getY0());
            clockwise = false;
        } else {
            float xc = cos( this->getTh0() + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin( this->getTh0() + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = Point(xc +this->getX0(),yc + this->getY0());
            clockwise = true;
        }

        start_point = p1;
        end_point = p2;

    }
}

// Get final x method
double DubinsArc::getXf() const {
	return this->xf;
}

// Get final y method
double DubinsArc::getYf() const {
	return this->yf;
}

// Get final angle method
double DubinsArc::getThf() const {
	return this->thf;
}

// Get lenght method
double DubinsArc::getL() const {
	return this->l;
}

// Get curvature method
double DubinsArc::getK() const {
	return this->k;
}

// Get initial x method
double DubinsArc::getX0() const {
    return this->x0;
}

// Get initial y method
double DubinsArc::getY0() const {
    return this->y0;
}

// Get initial angle method
double DubinsArc::getTh0() const {
	return this->th0;
}

// Print Method
void DubinsArc::print() const {
	cout << "Dubins arc x0: " << x0 << " y0: " << y0 << " th0: " << th0 << " k: " << k << " l: " << l << " xf " << xf << " yf: " << yf << " thf: " << thf << endl;
}
