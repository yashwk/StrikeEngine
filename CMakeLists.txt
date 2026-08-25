cmake_minimum_required(VERSION 3.23)
project(StrikeEngine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
include(FetchContent)

# Optional GPU compute backend. Any consumer that wants Vulkan executes it on
# GPU must build with this ON; core simulations are CPU-only by default.
option(STRIKEENGINE_WITH_VULKAN "Build the optional Vulkan compute backend" OFF)

FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.2
)

FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.12.0
)

FetchContent_MakeAvailable(glm)
FetchContent_MakeAvailable(json)

# New Architecture Sources
file(GLOB_RECURSE KERNEL_SOURCES "kernel/*.cpp")
list(FILTER KERNEL_SOURCES EXCLUDE REGEX ".*/backend/vulkan/.*")
file(GLOB_RECURSE MODELS_SOURCES "models/*.cpp")
file(GLOB_RECURSE SIM_SOURCES "simulation/*.cpp")

add_library(strikeengine STATIC
    ${KERNEL_SOURCES}
    ${MODELS_SOURCES}
    ${SIM_SOURCES}
)

target_include_directories(strikeengine PUBLIC "${CMAKE_SOURCE_DIR}")

target_link_libraries(strikeengine PUBLIC
    glm
    nlohmann_json
)

# --- Optional Vulkan compute backend -------------------------------------
# Requesting BackendType::Vulkan from SimulationKernel works only when the
# companion strikeengine_vulkan library is linked (it self-registers through
# kernel/backend/BackendFactory). Building without it throws a clear error.
if(STRIKEENGINE_WITH_VULKAN)
    find_package(Vulkan REQUIRED)

    add_library(strikeengine_vulkan STATIC
        kernel/backend/vulkan/VulkanContext.cpp
        kernel/backend/vulkan/VulkanBackend.cpp
    )

    target_include_directories(strikeengine_vulkan PRIVATE "${CMAKE_SOURCE_DIR}")
    target_link_libraries(strikeengine_vulkan PUBLIC Vulkan::Vulkan)

    # Compile Compute Shader
    add_custom_command(
        TARGET strikeengine_vulkan PRE_BUILD
        COMMAND glslc ${CMAKE_SOURCE_DIR}/kernel/backend/vulkan/shaders/physics_step.comp -o ${CMAKE_BINARY_DIR}/physics_step.spv
        COMMENT "Compiling compute shader to SPIR-V"
    )
endif()

add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

# Subdirectories
enable_testing()
add_subdirectory(tests)
