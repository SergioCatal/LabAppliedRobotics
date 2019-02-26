#include "Shape.hpp"

// Empty constructor
Shape::Shape() {
    center = Point(0,0);
}

// Full constructor
Shape::Shape(Point2f center) {
    this->center = center;
}

// Set method for the center
void Shape::setCenter(Point2f center) {
    this->center = center;
}

// Get method for the center
Point2f Shape::getCenter() const {
    return center;
}