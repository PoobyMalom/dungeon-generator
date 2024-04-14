#include <iostream>
#include <SDL2/SDL.h>
#include <vector>
#include "include/Utility.h"

using namespace std;

int main() 
{
    srand(time(NULL));

    // SDL Initialization
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        cerr << "SDL initialization failed: " << SDL_GetError() << endl;
        return 1;
    }

    // Create SDL window and renderer
    const int winWidth = 1000;
    const int winHeight = 1000;
    SDL_Window *win = SDL_CreateWindow("SDL Window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, winWidth, winHeight, 0);
    if (win == nullptr) {
        cerr << "Failed to create SDL window: " << SDL_GetError() << endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (ren == nullptr) {
        cerr << "Failed to create SDL renderer: " << SDL_GetError() << endl;
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    // Game loop
    bool quit = false;
    const int numPoints = 2;
    const double radius = 100.0;
    const int max_width = 100;
    const int max_height = 100;
    const int min_width = 20;
    const int min_height = 20;

    vector<Point> points = get_points(radius, numPoints);
    vector<SDL_Rect> rects = get_rects(points, max_height, max_width, numPoints, winHeight, winWidth, min_height, min_width);

    SDL_Event e;
    while (!quit) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        for (size_t i = 0; i < numPoints; ++i) {
            vector<int> pos = get_push_back(rects[i], rects, radius, numPoints, i);
            
        }

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);
        SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        for (int i = 0; i < numPoints; ++i) {
            SDL_RenderDrawRect(ren, &rects[i]);
        }
        SDL_RenderPresent(ren);
        SDL_Delay(100);
    }

    // Cleanup
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();

    return 0;
}
