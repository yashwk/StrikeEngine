# apps/StrikeDesigner/CMakeLists.txt
#
# Builds the 'StrikeDesigner' executable.

# --- Create a Helper Library for Dear ImGui ---
# This is a more robust and modern approach than globbing sources directly
# into the executable. It creates a static library that encapsulates all of
# ImGui's implementation details.
add_library(imgui_backend STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp"
)

# Make the necessary ImGui headers available to our backend library.
target_include_directories(imgui_backend PUBLIC
        "${imgui_SOURCE_DIR}/backends"
        "${imgui_SOURCE_DIR}"
)

# --- Define the Main Executable ---
add_executable(StrikeDesigner
        main.cpp
        # Note: We no longer list the ImGui sources here.
)

# Link the executable against its direct dependencies.
# Our new helper library is now a dependency.
target_link_libraries(StrikeDesigner PRIVATE
        strikeengine
        glfw
        imgui           # The main ImGui interface library from FetchContent
        imgui_backend   # Our new helper library with the compiled sources
        Vulkan::Vulkan
)

# Set the folder property for better organization in IDEs.
set_target_properties(StrikeDesigner PROPERTIES FOLDER "Applications")
