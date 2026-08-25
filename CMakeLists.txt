cmake_minimum_required(VERSION 3.23)
project(StrikeEngine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
include(FetchContent)

find_package(Vulkan REQUIRED)

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
file(GLOB_RECURSE MODELS_SOURCES "models/*.cpp")
file(GLOB_RECURSE SIM_SOURCES "simulation/*.cpp")

add_library(strikeengine STATIC
    ${KERNEL_SOURCES}
    ${MODELS_SOURCES}
    ${SIM_SOURCES}
)

# Compile Compute Shader
add_custom_command(
    TARGET strikeengine PRE_BUILD
    COMMAND glslc ${CMAKE_SOURCE_DIR}/kernel/backend/vulkan/shaders/physics_step.comp -o ${CMAKE_BINARY_DIR}/physics_step.spv
    COMMENT "Compiling compute shader to SPIR-V"
)

target_include_directories(strikeengine PUBLIC "${CMAKE_SOURCE_DIR}")

target_link_libraries(strikeengine PUBLIC
    glm
    nlohmann_json
    Vulkan::Vulkan
)

# Subdirectories
add_subdirectory(tests)
add_subdirectory(apps)

add_definitions(-DGLM_ENABLE_EXPERIMENTAL)
