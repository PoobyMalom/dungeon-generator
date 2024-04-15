#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <SDL2/SDL.h>
using namespace std;

struct xy {
    double x, y;
    xy(double x, double y);
};

struct polygon {
    vector<xy> vertex;
    vector<xy> edge;

    polygon(const vector<xy>& vertices, const vector<xy>& edges);
};

polygon rectToPolygon(const SDL_Rect& rect);
bool sat(const polygon& polygonA, const polygon& polygonB);

#endif // COLLISION_H