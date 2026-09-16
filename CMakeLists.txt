cmake_minimum_required(VERSION 3.23)
project(StrikeEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings for our own targets (add_compile_options applies from this
# directory down; FetchContent deps are header-only interface targets).
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

# A bare configure must not silently produce an unoptimized cache: presets
# provide the optimized dev (RelWithDebInfo) and release configurations; the
# implicit default stays assert-friendly.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# Dependencies
include(FetchContent)

# Optional GPU compute backend. Any consumer that wants Vulkan executes it on
# GPU must build with this ON; core simulations are CPU-only by default.
option(STRIKEENGINE_WITH_VULKAN "Build the optional Vulkan compute backend" OFF)
option(STRIKEENGINE_WITH_GDAL "Build GDAL-backed terrain loading" OFF)
option(STRIKEENGINE_BUILD_TESTS "Build the StrikeEngine test suite" ON)

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

# Public API layout: headers under include/strikeengine, implementation under
# src/strikeengine. Consumers use #include <strikeengine/...>.
file(GLOB_RECURSE STRIKEENGINE_SOURCES CONFIGURE_DEPENDS "src/strikeengine/*.cpp")
list(FILTER STRIKEENGINE_SOURCES EXCLUDE REGEX ".*/backend/vulkan/.*")

add_library(strikeengine STATIC ${STRIKEENGINE_SOURCES})

target_include_directories(strikeengine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Private, build-only headers shared between library translation units (for
# example the aero schema parser used by both the inline config and the profile
# loader). Kept out of include/ so their dependencies are not installed.
target_include_directories(strikeengine PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_compile_features(strikeengine PUBLIC cxx_std_23)

target_link_libraries(strikeengine PUBLIC
    $<BUILD_INTERFACE:glm>
    $<BUILD_INTERFACE:nlohmann_json>
)

if(STRIKEENGINE_WITH_GDAL)
    find_package(GDAL CONFIG REQUIRED)
    target_link_libraries(strikeengine PUBLIC GDAL::GDAL)
    target_compile_definitions(strikeengine PRIVATE STRIKEENGINE_WITH_GDAL=1)
endif()
# glm and nlohmann/json are header-only and used only in .cpp files, never in
# public headers. Build-tree consumers (add_subdirectory / FetchContent) get
# them via BUILD_INTERFACE. GDAL is an explicit dependency of installed
# packages built with STRIKEENGINE_WITH_GDAL=ON.

# --- Optional Vulkan compute backend -------------------------------------
# Requesting BackendType::Vulkan from SimulationKernel works only when the
# companion strikeengine_vulkan library is linked (it self-registers through
# strikeengine/kernel/backend/BackendFactory). Building without it throws a
# clear error.
if(STRIKEENGINE_WITH_VULKAN)
    find_package(Vulkan REQUIRED COMPONENTS glslc)

    add_library(strikeengine_vulkan STATIC
        src/strikeengine/kernel/backend/vulkan/VulkanContext.cpp
        src/strikeengine/kernel/backend/vulkan/VulkanBackend.cpp
    )

    target_include_directories(strikeengine_vulkan PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(strikeengine_vulkan PUBLIC Vulkan::Vulkan)

    # Compile Compute Shader
    add_custom_command(
        TARGET strikeengine_vulkan PRE_BUILD
        COMMAND Vulkan::glslc ${CMAKE_CURRENT_SOURCE_DIR}/src/strikeengine/kernel/backend/vulkan/shaders/physics_step.comp -o ${CMAKE_BINARY_DIR}/physics_step.spv
        COMMENT "Compiling compute shader to SPIR-V"
    )
endif()

add_definitions(-DGLM_ENABLE_EXPERIMENTAL)

# ---------------------------------------------------------------------------
# Install / package (Phase 1c): consumers use find_package(StrikeEngine) or
# FetchContent of a pinned tag. Headers install to include/, the static lib and
# (optionally) the Vulkan companion install as targets export, and a CMake
# package config makes the dependencies resolvable (see cmake/strikeengineConfig.cmake.in).
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS strikeengine
        EXPORT strikeengineTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/strikeengine
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(STRIKEENGINE_WITH_VULKAN)
    install(TARGETS strikeengine_vulkan
            EXPORT strikeengineTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(EXPORT strikeengineTargets
        FILE strikeengineTargets.cmake
        NAMESPACE StrikeEngine::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/strikeengine)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/StrikeEngineConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/strikeengineConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/StrikeEngineConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/strikeengine
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/StrikeEngineConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/StrikeEngineConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/strikeengine
)

# Subdirectories
if(STRIKEENGINE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
