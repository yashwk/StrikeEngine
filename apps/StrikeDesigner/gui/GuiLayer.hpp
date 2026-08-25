#pragma once

#include <SDL3/SDL.h>
#include <string>
#include <vector>
#include <mutex>
#include "../core/VehicleModel.hpp"

class VulkanContext;

struct UIState {
    bool showLibrary = true;
    bool showGraph = true;
    bool showInspector = true;
    bool showViewport = true;
    bool showConsole = true;
    bool showErrors = true;
    bool showOutput = true;
    bool showPlots = true;

    bool isSimulationRunning = false;
    float simulationTime = 0.0f;
    int selectedObjectId = -1;
};

// Simple Log Entry structure
struct LogEntry {
    std::string message;
    int type; // 0=Info, 1=Warn, 2=Error
    double timestamp;
};

class GuiLayer {
public:
    bool init(SDL_Window* window, VulkanContext& vk);
    void shutdown();

    void beginFrame();
    void render();
    void endFrame();

    void processEvent(const SDL_Event& e);

    UIState& getState() { return state; }
    Project& getProject() { return project; }

    // Static Logging API
    static void Log(const std::string& msg, int type = 0);
    static const std::vector<LogEntry>& GetLogs();
    static void ClearLogs();

private:
    UIState state;
    Project project;

    // Static log storage
    static std::vector<LogEntry> s_Logs;
};