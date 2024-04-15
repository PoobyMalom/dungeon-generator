#include <vector>
#include <algorithm>
#include <limits>
#include <iostream>
#include <SDL2/SDL.h>
using namespace std;

struct xy {
    double x, y;
    xy(double x, double y) : x(x), y(y) {}
};

struct polygon {
    vector<xy> vertex;
    vector<xy> edge;

    polygon(const vector<xy>& vertices, const vector<xy>& edges) : vertex(vertices), edge(edges) {}
};


polygon rectToPolygon(const SDL_Rect& rect) {
    vector<xy> vertices;
    vector<xy> edges;

    // Add vertices
    vertices.push_back(xy(rect.x, rect.y));
    vertices.push_back(xy(rect.x + rect.w, rect.y));
    vertices.push_back(xy(rect.x + rect.w, rect.y + rect.h));
    vertices.push_back(xy(rect.x, rect.y + rect.h));

    // Add edges
    edges.push_back(xy(rect.w, 0));
    edges.push_back(xy(0, rect.h));
    edges.push_back(xy(-rect.w, 0));
    edges.push_back(xy(0, -rect.h));

    return polygon(vertices, edges);
}

bool sat(const polygon& polygonA, const polygon& polygonB) {
    vector<xy> perpendicularStack;
    xy perpendicularLine(0, 0);
    double dot = 0;
    double amin, amax, bmin, bmax;
    double buffer = 0.1; // Adjusted buffer to 0.1 as per your requirement

    for (const auto& edge : polygonA.edge) {
        perpendicularLine = xy(-edge.y, edge.x);
        perpendicularStack.push_back(perpendicularLine);
    }

    for (const auto& edge : polygonB.edge) {
        perpendicularLine = xy(-edge.y, edge.x);
        perpendicularStack.push_back(perpendicularLine);
    }

    for (const auto& perp : perpendicularStack) {
        amin = numeric_limits<double>::max();
        amax = numeric_limits<double>::min();
        bmin = numeric_limits<double>::max();
        bmax = numeric_limits<double>::min();

        for (const auto& vertex : polygonA.vertex) {
            dot = vertex.x * perp.x + vertex.y * perp.y;
            amin = min(dot, amin);
            amax = max(dot, amax);
        }

        for (const auto& vertex : polygonB.vertex) {
            dot = vertex.x * perp.x + vertex.y * perp.y;
            bmin = min(dot, bmin);
            bmax = max(dot, bmax);
        }

        if (!(amin <= bmax + buffer && amax >= bmin - buffer)) {
            return false;
        }
    }

    return true;
}