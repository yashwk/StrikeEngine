# --- Create the Executable Target ---
add_executable(StrikeDesigner main.cpp)

# --- Add Preprocessor Definitions for ImGui ---
# This tells the compiler to enable the Docking and Viewport features.
target_compile_definitions(StrikeDesigner PRIVATE IMGUI_HAS_DOCK=1 IMGUI_HAS_VIEWPORT=1)

# --- Link Core Engine Library ---
target_link_libraries(StrikeDesigner PRIVATE strikeengine)

# --- Configure and Link UI Libraries ---
target_include_directories(StrikeDesigner PRIVATE
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
        ${glfw_SOURCE_DIR}/include
)

# --- Add ImGui Source Files ---
# We compile only the necessary source files directly into our executable.
target_sources(StrikeDesigner PRIVATE
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp # <-- FIX: Added the missing demo file.
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

# --- Final Linking ---
target_link_libraries(StrikeDesigner PRIVATE glfw Vulkan::Vulkan)
