#include "LibraryPanel.hpp"
#include <imgui.h>
#include "../../core/VehicleModel.hpp"

void LibraryPanel::draw(Project& project, bool* p_open)
{
	if (ImGui::Begin("Parts Library", p_open))
	{
		ImGui::TextDisabled("Click to add to: %s", project.selectedPart ? project.selectedPart->name.c_str() : "None");
		ImGui::Separator();

		if (ImGui::TreeNodeEx("Structures", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Selectable("Nose Cone (Ogive)")) {
				if (project.selectedPart) {
					auto part = std::make_shared<NoseCone>();
					project.selectedPart->addChild(part);
				}
			}
			if (ImGui::Selectable("Body Tube (Standard)")) {
				if (project.selectedPart) {
					auto part = std::make_shared<BodyTube>();
					project.selectedPart->addChild(part);
				}
			}
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Aerodynamics", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Selectable("Fin Set (Trapezoidal)")) {
				if (project.selectedPart) {
					auto part = std::make_shared<FinSet>();
					project.selectedPart->addChild(part);
				}
			}
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Propulsion", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Selectable("Solid Motor")) {
				if (project.selectedPart) {
					auto part = std::make_shared<Motor>();
					project.selectedPart->addChild(part);
				}
			}
			ImGui::TreePop();
		}
	}
	ImGui::End();
}