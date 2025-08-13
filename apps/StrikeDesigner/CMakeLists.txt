add_library(imgui_backend STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp"
)

target_include_directories(imgui_backend PUBLIC
        "${imgui_SOURCE_DIR}/backends"
        "${imgui_SOURCE_DIR}"
)
add_executable(StrikeDesigner
        main.cpp
)

target_link_libraries(StrikeDesigner PRIVATE
        strikeengine
        glfw
        imgui
        imgui_backend
        Vulkan::Vulkan
)

set_target_properties(StrikeDesigner PROPERTIES FOLDER "Applications")
