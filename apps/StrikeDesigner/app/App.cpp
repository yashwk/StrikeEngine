#include "App.hpp"

#include "../renderer/VulkanContext.hpp"
#include "../gui/GuiLayer.hpp"

#include <SDL3/SDL.h>
#include <cstdio>

// Global singletons
static VulkanContext g_vulkan;
static GuiLayer g_gui;

bool App::init()
{
    printf("=== StrikeDesigner INIT START ===\n");

    if (!initSDL()) {
        printf("App::init -> initSDL FAILED\n");
        return false;
    }

    printf("SDL initialized, window created\n");

    // --- Vulkan init ---
    printf("Initializing Vulkan...\n");
    if (!g_vulkan.init(window)) {
        printf("Vulkan init FAILED\n");
        return false;
    } else {
        printf("Vulkan init OK\n");
    }

    // --- GUI init ---
    printf("Initializing GUI...\n");
    if (!g_gui.init(window, g_vulkan)) {
        printf("GUI init FAILED\n");
        return false;
    }

    printf("GUI init OK\n");
    printf("=== StrikeDesigner INIT DONE ===\n");

    return true;
}

bool App::initSDL()
{
    printf("Calling SDL_Init(SDL_INIT_VIDEO)...\n");

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        printf("[SDL Init] FAILED: '%s'\n", SDL_GetError());
        return false;
    }

    printf("SDL Init OK\n");

    window = SDL_CreateWindow(
        "StrikeDesigner",
        1920,
        1080,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_VULKAN
    );

    if (!window) {
        printf("SDL_CreateWindow FAILED: '%s'\n", SDL_GetError());
        return false;
    }

    printf("SDL window created successfully\n");
    return true;
}

void App::run()
{
    printf("Entering main loop\n");

    while (running) {
        handleEvents();

        // 1. Acquire Image
        // If beginFrame returns false (minimized or resizing), skip the rest of the loop
        if (!g_vulkan.beginFrame(window)) {
            continue;
        }

        // 2. Build ImGui Draw Data
        g_gui.beginFrame();
        g_gui.render();
        g_gui.endFrame();

        // 3. Render Pass & Present
        g_vulkan.endFrame(window);
    }

    printf("Exiting main loop\n");
}

void App::handleEvents()
{
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_EVENT_QUIT) {
            running = false;
        }

        // Handle Resize Event explicitly
        if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
            g_vulkan.recreateSwapchain(window);
        }

        g_gui.processEvent(event);
    }
}

void App::shutdown()
{
    printf("Shutting down...\n");

    g_gui.shutdown();
    g_vulkan.shutdown();

    if (window) {
        SDL_DestroyWindow(window);
        window = nullptr;
    }

    SDL_Quit();
    printf("Shutdown complete\n");
}