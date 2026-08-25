cmake_minimum_required(VERSION 3.24)
project(StrikeDesigner)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Vulkan REQUIRED)

include(FetchContent)

# Fetch Prebuilt SDL3 for MinGW
FetchContent_Declare(
  sdl3_prebuilt
  URL https://github.com/libsdl-org/SDL/releases/download/release-3.2.4/SDL3-devel-3.2.4-mingw.zip
)
FetchContent_MakeAvailable(sdl3_prebuilt)

add_library(SDL3::SDL3 SHARED IMPORTED)
set_target_properties(SDL3::SDL3 PROPERTIES
  IMPORTED_LOCATION "${sdl3_prebuilt_SOURCE_DIR}/x86_64-w64-mingw32/bin/SDL3.dll"
  IMPORTED_IMPLIB "${sdl3_prebuilt_SOURCE_DIR}/x86_64-w64-mingw32/lib/libSDL3.dll.a"
  INTERFACE_INCLUDE_DIRECTORIES "${sdl3_prebuilt_SOURCE_DIR}/x86_64-w64-mingw32/include"
)

# Fetch ImGui Docking Branch
FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG docking
)
FetchContent_MakeAvailable(imgui)

add_executable(StrikeDesigner)

# --- App Entry ---
target_sources(StrikeDesigner PRIVATE
        app/main.cpp
        app/App.cpp
        renderer/VulkanContext.cpp
        core/VehicleModel.cpp
)
target_compile_definitions(StrikeDesigner PRIVATE SDL_MAIN_HANDLED)

add_custom_command(TARGET StrikeDesigner POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${sdl3_prebuilt_SOURCE_DIR}/x86_64-w64-mingw32/bin/SDL3.dll"
        $<TARGET_FILE_DIR:StrikeDesigner>
)

# --- GUI Layer ---
target_sources(StrikeDesigner PRIVATE
        gui/GuiLayer.cpp
        gui/Dockspace.cpp
        gui/MenuBar.cpp

        gui/panels/LibraryPanel.cpp
        gui/panels/GraphPanel.cpp
        gui/panels/InspectorPanel.cpp
        gui/panels/PlotPanel.cpp
        gui/panels/ViewportPanel.cpp
        gui/panels/ConsolePanel.cpp
        gui/panels/ErrorsPanel.cpp
        gui/panels/OutputPanel.cpp
)

# --- ImGui ---
target_sources(StrikeDesigner PRIVATE
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

# --- Include Paths ---
target_include_directories(StrikeDesigner PRIVATE
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
)

# --- Link Libraries ---
target_link_libraries(StrikeDesigner PRIVATE
        SDL3::SDL3
        Vulkan::Vulkan
        nlohmann_json
)
