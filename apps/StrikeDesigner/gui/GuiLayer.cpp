#include "../gui/GuiLayer.hpp"
#include "../gui/Dockspace.hpp"
#include "../gui/MenuBar.hpp"

#include "../renderer/VulkanContext.hpp"
#include "../gui/panels/GraphPanel.hpp"
#include "../gui/panels/InspectorPanel.hpp"
#include "../gui/panels/PlotPanel.hpp"
#include "../gui/panels/LibraryPanel.hpp"
#include "../gui/panels/ViewportPanel.hpp"
#include "../gui/panels/ConsolePanel.hpp"
#include "../gui/panels/ErrorsPanel.hpp"
#include "../gui/panels/OutputPanel.hpp"
#include "imgui.h"
#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>
#include <chrono>

static void applyStrikeStyle();

std::vector<LogEntry> GuiLayer::s_Logs;

void GuiLayer::Log(const std::string& msg, int type) {
	// Get time
	auto now = std::chrono::system_clock::now();
	double time = std::chrono::duration<double>(now.time_since_epoch()).count();

	s_Logs.push_back({msg, type, time});

	// Print to stdout as well
	if(type == 0) printf("[INFO] %s\n", msg.c_str());
	else if(type == 1) printf("[WARN] %s\n", msg.c_str());
	else printf("[ERR] %s\n", msg.c_str());
}

const std::vector<LogEntry>& GuiLayer::GetLogs() { return s_Logs; }
void GuiLayer::ClearLogs() { s_Logs.clear(); }

bool GuiLayer::init(SDL_Window* window, VulkanContext& vk)
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	applyStrikeStyle();

	GuiLayer::Log("StrikeDesigner System Initialising...",0);

	ImFontConfig fontCfg{};
	fontCfg.OversampleH = 3;
	fontCfg.OversampleV = 3;
	fontCfg.PixelSnapH = false;
	fontCfg.RasterizerMultiply = 1.2f;

	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->Clear();
	ImFont* uiFont = io.Fonts->AddFontFromFileTTF("data/fonts/Inter-Regular.ttf", 20.0f,&fontCfg);
	io.Fonts->AddFontDefault();

	io.FontGlobalScale = 1.0f;

	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

	ImGui_ImplSDL3_InitForVulkan(window);

	ImGui_ImplVulkan_InitInfo info{};
	vk.fillImGuiInitInfo(info);

    // --- REQUIRED for ImGui v1.91+ (Docking Branch) ---
    // If your compiler complains that 'PipelineInfoMain' is missing,
    // it means you are on an older version. In that case, use:
    // info.RenderPass = vk.getRenderPass();
    // info.Subpass = 0;
    // info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

	info.PipelineInfoMain.RenderPass = vk.getRenderPass();
	info.PipelineInfoMain.Subpass = 0;
	info.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

	// ---- Dynamic rendering OFF ----
	info.UseDynamicRendering = false;

	ImGui_ImplVulkan_Init(&info);

	GuiLayer::Log("Vulkan Backend Ready.", 0);
	return true;
}

void GuiLayer::shutdown()
{
	ImGui_ImplVulkan_Shutdown();
	ImGui_ImplSDL3_Shutdown();
	ImGui::DestroyContext();
}

void GuiLayer::processEvent(const SDL_Event& e)
{
	ImGui_ImplSDL3_ProcessEvent(&e);
}

void GuiLayer::beginFrame()
{
	ImGui_ImplVulkan_NewFrame();
	ImGui_ImplSDL3_NewFrame();
	ImGui::NewFrame();
}

void GuiLayer::render()
{
	Dockspace::begin();
	MainMenuBar::draw(*this);

	// Pass 'project' to the panels that need data access
	if (state.showLibrary)   LibraryPanel::draw(project, &state.showLibrary);
	if (state.showGraph)     GraphPanel::draw(project, &state.showGraph);
	if (state.showInspector) InspectorPanel::draw(project, &state.showInspector);

	// Others remain UI-only for now
	if (state.showViewport)  ViewportPanel::draw(&state.showViewport);
	if (state.showPlots)     PlotPanel::draw();
	if (state.showConsole)   ConsolePanel::draw(&state.showConsole);
	if (state.showErrors)    ErrorsPanel::draw();
	if (state.showOutput)    OutputPanel::draw();

	Dockspace::end();
}

void GuiLayer::endFrame()
{
	ImGui::Render();
	ImGuiIO& io = ImGui::GetIO();
	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	{
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
	}
}

static void applyStrikeStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    // 1. Geometry - Sharp, industrial look
    style.WindowRounding    = 2.0f;
    style.FrameRounding     = 2.0f;
    style.PopupRounding     = 2.0f;
    style.ScrollbarRounding = 2.0f;
    style.GrabRounding      = 2.0f;
    style.TabRounding       = 2.0f;

    style.WindowPadding     = ImVec2(10, 10);
    style.FramePadding      = ImVec2(8, 4);
    style.ItemSpacing       = ImVec2(8, 6);
    style.ScrollbarSize     = 14.0f;
	style.AntiAliasedFill   = true;
	style.AntiAliasedLines  = true;

    // 2. Core Palette
    // "Abyssal" Backgrounds (Near Pitch Black)
    colors[ImGuiCol_WindowBg]             = ImVec4(0.01f, 0.01f, 0.01f, 1.00f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.01f, 0.01f, 0.01f, 1.00f);
    colors[ImGuiCol_PopupBg]              = ImVec4(0.05f, 0.05f, 0.05f, 0.95f);
    colors[ImGuiCol_Border]               = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    // Headers & Titles
    colors[ImGuiCol_TitleBg]              = ImVec4(0.04f, 0.04f, 0.04f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.04f, 0.04f, 0.04f, 1.00f); // Keep title dark, let tabs pop
    colors[ImGuiCol_TitleBgCollapsed]     = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
    colors[ImGuiCol_MenuBarBg]            = ImVec4(0.025f, 0.025f, 0.025f, 1.00f);

    // Text (High Contrast)
    colors[ImGuiCol_Text]                 = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
    colors[ImGuiCol_TextDisabled]         = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);

    // 3. "Strike Orange" Accents
    // We use a strong, burnt orange
    auto strikeOrange       = ImVec4(1.0f, 0.3f, 0.00f, 1.00f); // Vibrant Safety Orange
    auto strikeOrangeDim    = ImVec4(1.0f, 0.45f, 0.00f, 1.00f);
    auto strikeOrangeBright = ImVec4(1.00f, 0.60f, 0.0f, 1.00f);

    // Headers (Used in Trees/Lists) - now Orange when selected
    colors[ImGuiCol_Header]               = ImVec4(strikeOrange.x, strikeOrange.y, strikeOrange.z, 0.20f); // Transparent orange
    colors[ImGuiCol_HeaderHovered]        = ImVec4(strikeOrange.x, strikeOrange.y, strikeOrange.z, 0.30f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(strikeOrange.x, strikeOrange.y, strikeOrange.z, 0.40f);

    // Buttons
    colors[ImGuiCol_Button]               = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = strikeOrangeDim; // Orange when clicking

    // Tabs - This is where the orange really pops
    colors[ImGuiCol_Tab]                  = ImVec4(0.12f, 0.12f, 0.12f, 1.00f);
    colors[ImGuiCol_TabHovered]           = strikeOrangeDim;
    colors[ImGuiCol_TabActive]            = strikeOrange;
    colors[ImGuiCol_TabUnfocused]         = ImVec4(0.10f, 0.10f, 0.10f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]   = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);

    // Interactive Elements
    colors[ImGuiCol_CheckMark]            = strikeOrange;
    colors[ImGuiCol_SliderGrab]           = strikeOrange;
    colors[ImGuiCol_SliderGrabActive]     = strikeOrangeBright;
    colors[ImGuiCol_Separator]            = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_SeparatorHovered]     = strikeOrange;
    colors[ImGuiCol_SeparatorActive]      = strikeOrange;
    colors[ImGuiCol_ResizeGrip]           = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    colors[ImGuiCol_ResizeGripHovered]    = strikeOrange;
    colors[ImGuiCol_ResizeGripActive]     = strikeOrange;

    // Plots/Graphs
    colors[ImGuiCol_PlotLines]            = strikeOrange;
    colors[ImGuiCol_PlotLinesHovered]     = strikeOrangeBright;
    colors[ImGuiCol_PlotHistogram]        = strikeOrange;
    colors[ImGuiCol_PlotHistogramHovered] = strikeOrangeBright;

    // Docking
    colors[ImGuiCol_DockingPreview]       = ImVec4(strikeOrange.x, strikeOrange.y, strikeOrange.z, 0.30f);
    colors[ImGuiCol_DockingEmptyBg]       = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
}