#include <iostream>

// --- Library Headers ---
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

// --- Engine Headers ---
#include "strikeengine/designer/VehicleModel.hpp"

void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

int main(int, char**) {
    // --- 1. Initialize GLFW and Create a Window ---
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "StrikeDesigner", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }

    // --- 2. Initialize Dear ImGui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // This will now be recognized
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // This will now be recognized

    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // --- 3. Initialize Vulkan Renderer and ImGui Backends (Placeholder) ---
    std::cout << "Placeholder: Initialize Vulkan Renderer here." << std::endl;
    // ImGui_ImplGlfw_InitForVulkan(window, true);
    // ImGui_ImplVulkan_Init(...);

    // --- 4. Main Application Loop ---
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui::NewFrame();

        // --- Our UI Code Goes Here ---
        ImGui::Begin("Hello, StrikeDesigner!");
        ImGui::Text("Docking and Viewports are now enabled.");
        ImGui::End();

        ImGui::Render();

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
    }

    // --- 5. Cleanup ---
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
