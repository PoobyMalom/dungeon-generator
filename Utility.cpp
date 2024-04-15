#include "include/Utility.h"
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "Collision.h"
#include <stack>
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

vector<int> get_push_back(const SDL_Rect& rect, vector<SDL_Rect>& rects, int numPoints, int index)
{
    vector<int> pos(2, 0); // Initialize position vector with zeros
    
    for (int i = index+1; i < numPoints; i++) 
    {
        SDL_Rect temp_rect = rects[i];
        PenetrationAxis collisionAxis = CheckCollision(rect, temp_rect);
        if (collisionAxis == PenetrationAxis::X) {
            pos[0] += 2 * rect.w/xdistance(rect, temp_rect);
        } else if (collisionAxis == PenetrationAxis::Y) {
            pos[1] += 2 * rect.h/ydistance(rect, temp_rect);
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

void updateAgents(vector<Agent>& agents, int winWidth, int winHeight, double radius) {
    double scale = 0.05; // Increase the scale of the separation force
    double cohesionScale = 0.01;
    double boundaryScale = 0.01;
    double maxVelocity = 5.0;
    double boundaryThreshold = 50.0;

    double centerX = 0;
    double centerY = 0;
    for (Agent& A : agents) {
        centerX += A.rect.x;
        centerY += A.rect.y;
    }
    centerX /= agents.size();
    centerY /= agents.size();

    for (Agent& A : agents) {
        vector<double> separationForce(2, 0);
        bool isOverlapping = false;
        for (Agent& B : agents) {
            if (&A != &B) {
                polygon polyA = rectToPolygon(A.rect);
                polygon polyB = rectToPolygon(B.rect);
                if (sat(polyA, polyB)) {
                    vector<double> force = {static_cast<double>(A.rect.x) - B.rect.x, static_cast<double>(A.rect.y) - B.rect.y};
                    double invDistance = 1.0 / sqrt(force[0]*force[0] + force[1]*force[1]);
                    force[0] *= invDistance;
                    force[1] *= invDistance;
                    separationForce[0] += force[0];
                    separationForce[1] += force[1];
                    isOverlapping = true;
                }
            }
        }

        if (!isOverlapping) {
        // Move towards the center of mass
            A.velocity[0] = (centerX - A.rect.x) * cohesionScale;
            A.velocity[1] = (centerY - A.rect.y) * cohesionScale;
        } else {
            if (abs(separationForce[0]) > abs(separationForce[1])) {
                separationForce[1] = 0;
            } else {
                separationForce[0] = 0;
            }

            A.velocity[0] += separationForce[0] * scale;
            A.velocity[1] += separationForce[1] * scale;

            double distanceToCenterSquared = (A.rect.x - centerX) * (A.rect.x - centerX) + (A.rect.y - centerY) * (A.rect.y - centerY);
            if (distanceToCenterSquared > radius * radius) {
                A.velocity[0] += (centerX - A.rect.x) * cohesionScale;
                A.velocity[1] += (centerY - A.rect.y) * cohesionScale;
            }

            if (A.rect.x < boundaryThreshold) {
                A.velocity[0] += boundaryScale;
            } else if (A.rect.x + A.rect.w > winWidth - boundaryThreshold) {
                A.velocity[0] -= boundaryScale;
            }

            if (A.rect.y < boundaryThreshold) {
                A.velocity[1] += boundaryScale;
            } else if (A.rect.y + A.rect.h > winHeight - boundaryThreshold) {
                A.velocity[1] -= boundaryScale;
            }

            double velocityMagnitudeSquared = A.velocity[0]*A.velocity[0] + A.velocity[1]*A.velocity[1];
            if (velocityMagnitudeSquared > maxVelocity * maxVelocity) {
                double invVelocityMagnitude = 1.0 / sqrt(velocityMagnitudeSquared);
                A.velocity[0] *= invVelocityMagnitude * maxVelocity;
                A.velocity[1] *= invVelocityMagnitude * maxVelocity;
            }
        }

        A.rect.x += A.velocity[0];
        A.rect.y += A.velocity[1];
    }
}

bool isSingleMass(vector<Agent>& agents) {
    int n = agents.size();
    vector<vector<int>> adj(n); // adjacency list
    vector<bool> visited(n, false);

    // Build the adjacency list
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            polygon polyA = rectToPolygon(agents[i].rect);
            polygon polyB = rectToPolygon(agents[j].rect);
            if (sat(polyA, polyB)) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

    // Depth-first search
    stack<int> s;
    s.push(0);
    visited[0] = true;
    while (!s.empty()) {
        int v = s.top();
        s.pop();
        for (int u : adj[v]) {
            if (!visited[u]) {
                s.push(u);
                visited[u] = true;
            }
        }
    }

    // Check if all rectangles were visited
    for (bool v : visited) {
        if (!v) {
            return false;
        }
    }

    return true;
}

void moveSmallestGroup(vector<Agent>& agents, int winWidth, int winHeight) {
    int n = agents.size();
    vector<vector<int>> adj(n); // adjacency list
    vector<bool> visited(n, false);
    vector<vector<int>> groups;

    // Build the adjacency list
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            polygon polyA = rectToPolygon(agents[i].rect);
            polygon polyB = rectToPolygon(agents[j].rect);
            if (sat(polyA, polyB)) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

    // Depth-first search to find the connected components
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            stack<int> s;
            s.push(i);
            visited[i] = true;
            vector<int> group;
            while (!s.empty()) {
                int v = s.top();
                s.pop();
                group.push_back(v);
                for (int u : adj[v]) {
                    if (!visited[u]) {
                        s.push(u);
                        visited[u] = true;
                    }
                }
            }
            groups.push_back(group);
        }
    }

    // Find the smallest group and the center of the larger group
    int minSize = n;
    int minIndex = -1;
    double centerX = 0.0;
    double centerY = 0.0;
    for (int i = 0; i < groups.size(); ++i) {
        if (groups[i].size() < minSize) {
            minSize = groups[i].size();
            minIndex = i;
        } else {
            for (int j : groups[i]) {
                centerX += agents[j].rect.x;
                centerY += agents[j].rect.y;
            }
        }
    }
    centerX /= (n - minSize);
    centerY /= (n - minSize);

    // Move the rectangles in the smallest group towards the center of the larger group
    for (int i : groups[minIndex]) {
        double dx = centerX - agents[i].rect.x;
        double dy = centerY - agents[i].rect.y;
        double dist = sqrt(dx*dx + dy*dy);
        agents[i].rect.x += dx / dist;
        agents[i].rect.y += dy / dist;
    }
}

void centerRects(vector<Agent>& agents, int centerX, int centerY) {
    for (Agent& agent : agents) {
        // Calculate the direction vector from the agent to the center point
        int dx = centerX - agent.rect.x;
        int dy = centerY - agent.rect.y;

        // Normalize the direction vector
        double length = sqrt(dx * dx + dy * dy);
        dx /= length;
        dy /= length;

        // Move the agent slightly towards the center point
        agent.rect.x += dx;
        agent.rect.y += dy;
    }
}