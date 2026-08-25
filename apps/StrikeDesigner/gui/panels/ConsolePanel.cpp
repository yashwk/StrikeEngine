#include "ConsolePanel.hpp"
#include <imgui.h>
#include "../GuiLayer.hpp"

void ConsolePanel::draw(bool* p_open)
{
    if (ImGui::Begin("Console", p_open))
    {
        // Toolbar
        if (ImGui::Button("Clear")) { GuiLayer::ClearLogs(); }
        ImGui::SameLine();

        static char filter[64] = "";
        ImGui::SetNextItemWidth(150);
        ImGui::InputTextWithHint("##filter", "Filter logs...", filter, sizeof(filter));

        ImGui::Separator();

        // Logs
        const float footer_height_to_reserve = ImGui::GetStyle().ItemSpacing.y + ImGui::GetFrameHeightWithSpacing();
        ImGui::BeginChild("ScrollingRegion", ImVec2(0, -footer_height_to_reserve), false, ImGuiWindowFlags_HorizontalScrollbar);

        const auto& logs = GuiLayer::GetLogs();
        for (const auto& log : logs) {
            // Simple filter check
            if (filter[0] != '\0' && log.message.find(filter) == std::string::npos)
                continue;

            ImVec4 col = ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
            const char* prefix = "[INFO]";

            if (log.type == 1) { col = ImVec4(1.0f, 0.8f, 0.2f, 1.0f); prefix = "[WARN]"; }
            if (log.type == 2) { col = ImVec4(1.0f, 0.4f, 0.4f, 1.0f); prefix = "[ERR ]"; }

            ImGui::TextColored(col, "%s %s", prefix, log.message.c_str());
        }

        // Auto-scroll on new items
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndChild();

        // Command Line
        ImGui::Separator();
        static char inputBuf[256] = "";
        if (ImGui::InputText("Command", inputBuf, sizeof(inputBuf), ImGuiInputTextFlags_EnterReturnsTrue))
        {
            GuiLayer::Log(std::string("Command not recognized: ") + inputBuf, 1);
            memset(inputBuf, 0, sizeof(inputBuf));
            ImGui::SetKeyboardFocusHere(-1);
        }
    }
    ImGui::End();
}