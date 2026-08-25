#include <imgui.h>
#include "PlotPanel.hpp"

namespace PlotPanel {
	void draw()
	{
		ImGui::Begin("Plots");
		ImGui::Text("Thrust, mass, aero curves");
		ImGui::End();
	}
}
