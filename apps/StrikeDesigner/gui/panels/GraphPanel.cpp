#include "GraphPanel.hpp"
#include <imgui.h>
#include "../../core/VehicleModel.hpp"

// Recursive function to draw tree nodes
void drawPartNode(RocketPart* part, Project& project) {
	ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

	// Highlight if selected
	if (project.selectedPart == part) {
		flags |= ImGuiTreeNodeFlags_Selected;
	}

	// Leaf node check
	if (part->children.empty()) {
		flags |= ImGuiTreeNodeFlags_Leaf;
	} else {
		flags |= ImGuiTreeNodeFlags_DefaultOpen;
	}

	bool isOpen = ImGui::TreeNodeEx((void*)part, flags, "%s", part->name.c_str());

	// Handle Click
	if (ImGui::IsItemClicked()) {
		project.select(part);
	}

	if (isOpen) {
		for (auto& child : part->children) {
			drawPartNode(child.get(), project);
		}
		ImGui::TreePop();
	}
}

void GraphPanel::draw(Project& project, bool* p_open)
{
	if (ImGui::Begin("Vehicle Graph", p_open))
	{
		if (project.root) {
			drawPartNode(project.root.get(), project);
		} else {
			ImGui::Text("No project loaded.");
		}
	}
	ImGui::End();
}