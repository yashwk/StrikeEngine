#include "InspectorPanel.hpp"
#include <imgui.h>
#include "../../core/VehicleModel.hpp" // Access to Part classes

void InspectorPanel::draw(Project& project, bool* p_open)
{
    if (ImGui::Begin("Inspector", p_open))
    {
        RocketPart* selection = project.selectedPart;

        if (!selection) {
            ImGui::TextDisabled("No part selected.");
        }
        else {
            // Header showing the type
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "Selected: %s", selection->name.c_str());
            ImGui::Separator();

            // Polymorphic dispatch: calling specific UI for the part type
            selection->onInspect();

            ImGui::Spacing();
            ImGui::Separator();

            // Delete Logic
            if (selection->type != PartType::Root) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.2f, 0.2f, 1.0f));
                if (ImGui::Button("Delete Part", ImVec2(-1, 0))) {
                    if (selection->parent) {
                        RocketPart* parent = selection->parent;
                        parent->removeChild(selection);
                        project.select(parent); // Select parent after delete
                    }
                }
                ImGui::PopStyleColor();
            }
        }
    }
    ImGui::End();
}