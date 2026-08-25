#include "MenuBar.hpp"
#include <imgui.h>
#include "GuiLayer.hpp"

void MainMenuBar::draw(GuiLayer& gui)
{
    UIState& state = gui.getState();

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("New Project", "Ctrl+N")) {
                gui.getProject() = Project();
            }
            if (ImGui::MenuItem("Open Project...", "Ctrl+O")) {
                gui.getProject().loadFromFile("workspace.strike");
            }
            if (ImGui::MenuItem("Save", "Ctrl+S")) {
                gui.getProject().saveToFile("workspace.strike");
            }
            if (ImGui::MenuItem("Save As")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Import Mesh")) {}
            if (ImGui::MenuItem("Export Design")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Alt+F4")) { /* Needs App close logic */ }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit"))
        {
            if (ImGui::MenuItem("Undo", "Ctrl+Z")) {}
            if (ImGui::MenuItem("Redo", "Ctrl+Y")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Cut", "Ctrl+X")) {}
            if (ImGui::MenuItem("Copy", "Ctrl+C")) {}
            if (ImGui::MenuItem("Paste", "Ctrl+V")) {}
            ImGui::Separator();
            if (ImGui::MenuItem("Project Settings")) {}
            if (ImGui::MenuItem("Preferences")) {}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Library", nullptr, &state.showLibrary);
            ImGui::MenuItem("Graph Hierarchy", nullptr, &state.showGraph);
            ImGui::MenuItem("Inspector", nullptr, &state.showInspector);
            ImGui::MenuItem("Viewport", nullptr, &state.showViewport);
            ImGui::MenuItem("Plots", nullptr, &state.showPlots);
            ImGui::Separator();
            ImGui::MenuItem("Console", nullptr, &state.showConsole);
            ImGui::MenuItem("Errors", nullptr, &state.showErrors);
            ImGui::MenuItem("Output", nullptr, &state.showOutput);
            ImGui::Separator();
            if (ImGui::MenuItem("Reset Layout")) {}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Simulation"))
        {
            if (ImGui::MenuItem("Run Simulation", "F5", state.isSimulationRunning)) {
                state.isSimulationRunning = !state.isSimulationRunning;
            }
            if (ImGui::MenuItem("Pause", "F6")) {
                state.isSimulationRunning = false;
            }
            if (ImGui::MenuItem("Stop", "Shift+F5")) {
                state.isSimulationRunning = false;
                state.simulationTime = 0.0f;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Analyze Aerodynamics")) {}
            if (ImGui::MenuItem("Compute Mass Properties")) {}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help"))
        {
            if (ImGui::MenuItem("Documentation")) {}
            if (ImGui::MenuItem("About StrikeDesigner")) {}
            ImGui::EndMenu();
        }

        // Simulation Status Indicator
        if (state.isSimulationRunning) {
             ImGui::SameLine(ImGui::GetWindowWidth() - 150);
             ImGui::TextColored(ImVec4(0,1,0,1), "SIMULATION RUNNING");
        }

        ImGui::EndMainMenuBar();
    }
}