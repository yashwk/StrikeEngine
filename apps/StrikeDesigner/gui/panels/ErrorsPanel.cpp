#include "imgui.h"
#include "ErrorsPanel.hpp"

namespace ErrorsPanel {

	void draw()
	{
		ImGui::Begin("Errors");
		ImGui::TextColored(ImVec4(1,0,0,1), "No errors");
		ImGui::End();
	}
}
