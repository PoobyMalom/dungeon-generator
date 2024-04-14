#include "include/Point.h"
#include <cmath>

Point::Point(double xVal, double yVal) : x(xVal), y(yVal) {}

double Point::getX() const {
    return x;
}

double Point::getY() const {
    return y;
}

void Point::setX(double newX) {
    x = newX;
}

void Point::setY(double newY) {
    y = newY;
}
