#include "include/Utility.h"
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
using namespace std;

Point get_random_point_in_circle(double radius)
{
    double t = 2 * (static_cast<double>(rand()) / RAND_MAX) * M_PI;
    double u = (static_cast<double>(rand()) / RAND_MAX) + (static_cast<double>(rand()) / RAND_MAX);
    double r = 0;
    if (u > 1) {
        r = 2 - u;
    } else {
        r = u;
    };
    return Point(radius * r * cos(t), radius * r * sin(t));
}

vector<Point> get_points(double radius, int numPoints)
{
    vector<Point> points;

    for (int i = 0; i < numPoints; ++i) {
        points.push_back(get_random_point_in_circle(radius));
    }

    return points;
}

vector<SDL_Rect> get_rects(vector<Point> points, int max_height, int max_width, int numPoints, int win_h, int win_w, int min_height, int min_width) 
{
    vector<SDL_Rect> rects;
    for (int i  = 0; i < numPoints; i++)
    {
        SDL_Rect rect;
        rect.x = points[i].getX() + (win_w / 2);
        rect.y = points[i].getY() + (win_h / 2); 
        rect.w = floor((static_cast<double>(rand()) / RAND_MAX) * (max_width - min_width) + min_width);
        rect.h = floor((static_cast<double>(rand()) / RAND_MAX) * (max_height - min_height) + min_height);
        rects.push_back(rect);
    }
    return rects;
}

double distance(const Point& point1, Point& point2)
{
    return sqrt(pow(point1.getX() - point2.getX(), 2) + pow(point1.getY() - point2.getY(), 2));
}

double xdistance(const SDL_Rect& rect1, SDL_Rect& rect2)
{
    return (rect1.x - rect2.x);
}

double ydistance(const SDL_Rect& rect1, SDL_Rect& rect2)
{
    return (rect1.y - rect2.y);
}

vector<int> get_push_back(const SDL_Rect& rect, vector<SDL_Rect>& rects, int radius, int numPoints, size_t index)
{
    vector<int> pos(2, 0); // Initialize position vector with zeros
    
    for (size_t i = index+1; i < numPoints; i++) 
    {
        SDL_Rect temp_rect = rects[i];
        PenetrationAxis collisionAxis = CheckCollision(rect, temp_rect);
        if (collisionAxis == PenetrationAxis::X) {
            pos[0] += xdistance(rect, temp_rect)/10;
        } else if (collisionAxis == PenetrationAxis::Y) {
            pos[1] += ydistance(rect, temp_rect)/10;
        }
    }
    return pos;   
}

SDL_Rect move_rect(SDL_Rect rect, int move_x, int move_y)
{
    SDL_Rect nrect;
    nrect.h = rect.h;
    nrect.w = rect.w;
    nrect.x = rect.x + move_x;
    nrect.y = rect.y + move_y;
    return nrect;
}

PenetrationAxis CheckCollision(const SDL_Rect& rectA, const SDL_Rect& rectB) {
    // Calculate overlap on the x-axis
    int overlapX = max(0, min(rectA.x + rectA.w, rectB.x + rectB.w) - max(rectA.x, rectB.x));
    // Calculate overlap on the y-axis
    int overlapY = max(0, min(rectA.y + rectA.h, rectB.y + rectB.h) - max(rectA.y, rectB.y));

    // Check if there is overlap
    if (overlapX > 0 && overlapY > 0) {
        // Determine which axis has the minimum penetration
        if (overlapX < overlapY) {
            return X;
        } else {
            return Y;
        }
    }

    // No collision
    return None;
}
