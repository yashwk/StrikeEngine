cmake_minimum_required(VERSION 3.23)
project(StrikeEngine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PROJECT_APPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apps)
set(PROJECT_TOOLS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tools)
set(PROJECT_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)
set(PROJECT_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/data)

# Dependencies
include(FetchContent)
find_package(Vulkan REQUIRED)

set(VULKAN_INCLUDE_DIR ${Vulkan_INCLUDE_DIRS})
set(VULKAN_LIBRARIES ${Vulkan_LIBRARIES})

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

FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.5-docking
)

FetchContent_MakeAvailable(glm)
FetchContent_MakeAvailable(json)
FetchContent_MakeAvailable(glfw)
FetchContent_MakeAvailable(imgui)

# Subdirectories
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tools)
add_subdirectory(tests)

#add_definitions(-DGLM_ENABLE_EXPERIMENTAL)
