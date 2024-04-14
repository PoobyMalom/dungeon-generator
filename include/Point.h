#ifndef POINT_H
#define POINT_H

class Point {
private:
    double x;
    double y;

public:
    Point(double xVal, double yVal);
    double getX() const;
    double getY() const;
    void setX(double newX);
    void setY(double newY);
};

enum PenetrationAxis {
    None,
    X,
    Y
};

#endif // POINT_H
