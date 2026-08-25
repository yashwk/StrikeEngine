#pragma once

#include <SDL3/SDL.h>

class App {
public:
    bool init();
    void run();
    void shutdown();

private:
    bool initSDL();
    void handleEvents();

private:
    SDL_Window* window = nullptr;
    bool running = true;
};
