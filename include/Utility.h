#ifndef UTILITY_H
#define UTILITY_H

#include "Point.h"
#include <vector>
#include <SDL2/SDL.h>

Point get_random_point_in_circle(double radius);
std::vector<Point> get_points(double radius, int numPoints);
std::vector<SDL_Rect> get_rects(std::vector<Point> points, int max_height, int max_width, int numPoints, int win_h, int win_w, int min_height, int min_width);
double distance(const Point& point1, Point& point2);
std::vector<int> get_push_back(const SDL_Rect& rect, std::vector<SDL_Rect>& rects, int radius, int numPoints, size_t index);
SDL_Rect move_rect(SDL_Rect rect, int move_x, int move_y);
PenetrationAxis CheckCollision(const SDL_Rect& rectA, const SDL_Rect& rectB);

#endif // UTILITY_H