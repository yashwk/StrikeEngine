#include "Dockspace.hpp"
#include <imgui.h>
#include "imgui_internal.h"

namespace Dockspace {

static bool firstFrame = true;

void buildLayout(ImGuiID dockspaceID)
{
    ImGui::DockBuilderRemoveNode(dockspaceID);
    ImGui::DockBuilderAddNode(dockspaceID, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspaceID, ImGui::GetMainViewport()->WorkSize);

    ImGuiID dockMain   = dockspaceID;
    ImGuiID dockLeft;
    ImGuiID dockRight;
    ImGuiID dockBottom;

    // 1. Create the Left Sidebar (18% width)
    ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Left,  0.18f, &dockLeft,  &dockMain);

    // 2. Create the Right Sidebar (22% width of the REMAINDER)
    // Note: Splitting the remainder means we need a slightly larger ratio to get true 22% of total width,
    // but 0.27f of the remainder is roughly correct visually for an Inspector.
    ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Right, 0.27f, &dockRight, &dockMain);

    // 3. Create the Bottom Console (20% height of the CENTRAL area)
    ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Down,  0.20f, &dockBottom,&dockMain);

    // 4. Assign Windows
    // Left
    ImGui::DockBuilderDockWindow("Parts Library", dockLeft);
    ImGui::DockBuilderDockWindow("Vehicle Graph", dockLeft);

    // Center (Main)
    ImGui::DockBuilderDockWindow("Viewport", dockMain);

    // Right
    ImGui::DockBuilderDockWindow("Inspector", dockRight);
    ImGui::DockBuilderDockWindow("Plots", dockRight);

    // Bottom
    ImGui::DockBuilderDockWindow("Console", dockBottom);
    ImGui::DockBuilderDockWindow("Errors", dockBottom);
    ImGui::DockBuilderDockWindow("Output", dockBottom);

    ImGui::DockBuilderFinish(dockspaceID);
}

void begin()
{
    // Host window flags
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus |
        ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_MenuBar;

    ImGuiViewport* vp = ImGui::GetMainViewport();

    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::SetNextWindowViewport(vp->ID);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGui::Begin("##RootWindow", nullptr, flags);

    ImGui::PopStyleVar(3);

    ImGuiID dockspaceID = ImGui::GetID("StrikeDesignerDockspace");

    // PassthruCentralNode allows the Vulkan clear color to show in the Viewport if no window is docked there
    ImGui::DockSpace(dockspaceID, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

    if (firstFrame) {
        buildLayout(dockspaceID);
        firstFrame = false;
    }
}

void end()
{
    ImGui::End();
}

}