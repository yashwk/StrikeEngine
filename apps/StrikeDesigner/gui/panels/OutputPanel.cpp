#include "imgui.h"
#include "OutputPanel.hpp"

namespace OutputPanel {
	void draw()
	{
		ImGui::Begin("Output");
		ImGui::Text("Output messages...");
		ImGui::End();
	}
}
