#include "Victim.hpp"

// Empty constructor
Victim::Victim() : Circle(){
    this->number = 0;
}

// Empty constructor
Victim::Victim(int number, Point2f center, float radius) : Circle(center,radius){
    this->number = number;
}


// Function that returns the number associated with the circle
int Victim::getNumber() const {
    return number;
}

// Set function for the number
void Victim::setNumber(int number) {
    this->number = number;
}

// Print function
void Victim::print() const {
    cout << "Circle center " <<  this->getCenter() << " radius " << this->getRadius() << " number " << number << endl;
}

// Comparison function
bool Victim::operator< (const Victim& other) {
    return this->number < other.getNumber();
}
