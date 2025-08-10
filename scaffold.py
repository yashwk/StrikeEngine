from pathlib import Path

# Define the full project structure based on the refactored layout.
# Directories must end with a forward slash '/'.
# Files should not.
PROJECT_STRUCTURE = [
    "apps/",
    "apps/MissionCLI/",
    "apps/MissionCLI/CMakeLists.txt",
    "apps/MissionCLI/main.cpp",
    "apps/StrikeDesigner/",
    "apps/StrikeDesigner/CMakeLists.txt",
    "apps/StrikeDesigner/DesignerUI.hpp",
    "apps/StrikeDesigner/main.cpp",
    "apps/StrikeSim/",
    "apps/StrikeSim/App.hpp",
    "apps/StrikeSim/CMakeLists.txt",
    "apps/StrikeSim/ScenarioLoader.hpp",
    "apps/StrikeSim/main.cpp",
    "cmake/",
    "cmake/VulkanSetup.cmake",
    "data/",
    "data/atmosphere_table.bin",
    "data/config/",
    "data/config/simulation_settings.json",
    "data/profiles/",
    "data/profiles/sa_missile_mk1.json",
    "data/profiles/aesa_tracker_v1.json",
    "data/rcs_profiles/",
    "data/rcs_profiles/f16_rcs.json",
    "data/scenarios/",
    "data/scenarios/intercept_test_01.json",
    "data/terrain/",
    "data/terrain/srtm_data/",
    "docs/",
    "docs/architecture.md",
    "docs/data_formats.md",
    "docs/developer_notes.md",
    "docs/modules.md",
    "docs/roadmap.md",
    "external/",
    "external/glm/",
    "include/",
    "include/strikeengine/",
    "include/strikeengine/atmosphere/",
    "include/strikeengine/atmosphere/AtmosphereModel.hpp",
    "include/strikeengine/atmosphere/AtmosphereTableLoader.hpp",
    "include/strikeengine/components/guidance/",
    "include/strikeengine/components/guidance/GuidanceComponent.hpp",
    "include/strikeengine/components/physics/",
    "include/strikeengine/components/physics/DragComponent.hpp",
    "include/strikeengine/components/physics/ForceAccumulator.hpp",
    "include/strikeengine/components/physics/LiftComponent.hpp",
    "include/strikeengine/components/physics/MassComponent.hpp",
    "include/strikeengine/components/physics/ThrustComponent.hpp",
    "include/strikeengine/components/physics/VelocityComponent.hpp",
    "include/strikeengine/components/transform/",
    "include/strikeengine/components/transform/TransformComponent.hpp",
    "include/strikeengine/core/",
    "include/strikeengine/core/Engine.hpp",
    "include/strikeengine/core/Profiler.hpp",
    "include/strikeengine/core/TimeManager.hpp",
    "include/strikeengine/ecs/",
    "include/strikeengine/ecs/Component.hpp",
    "include/strikeengine/ecs/Entity.hpp",
    "include/strikeengine/ecs/Registry.hpp",
    "include/strikeengine/ecs/System.hpp",
    "include/strikeengine/entities/",
    "include/strikeengine/entities/Material.hpp",
    "include/strikeengine/entities/Missile.hpp",
    "include/strikeengine/entities/Radar.hpp",
    "include/strikeengine/entities/SimEntity.hpp",
    "include/strikeengine/flight/",
    "include/strikeengine/flight/FlightModel.hpp",
    "include/strikeengine/guidance_laws/",
    "include/strikeengine/guidance_laws/GuidanceLaw.hpp",
    "include/strikeengine/guidance_laws/PNGuidance.hpp",
    "include/strikeengine/guidance_laws/RadarHoming.hpp",
    "include/strikeengine/io/",
    "include/strikeengine/io/ConfigLoader.hpp",
    "include/strikeengine/io/ProfileLoader.hpp",
    "include/strikeengine/io/ScenarioExporter.hpp",
    "include/strikeengine/physics/",
    "include/strikeengine/physics/Collision.hpp",
    "include/strikeengine/physics/Forces.hpp",
    "include/strikeengine/physics/Integrator.hpp",
    "include/strikeengine/physics/PhysicsSystem.hpp",
    "include/strikeengine/physics/State.hpp",
    "include/strikeengine/radar/",
    "include/strikeengine/radar/RCSDatabase.hpp",
    "include/strikeengine/radar/RadarSimulator.hpp",
    "include/strikeengine/scripting/",
    "include/strikeengine/scripting/MissionBindings.hpp",
    "include/strikeengine/scripting/ScriptEngine.hpp",
    "include/strikeengine/simulation/",
    "include/strikeengine/simulation/EntityManager.hpp",
    "include/strikeengine/simulation/ScenarioRunner.hpp",
    "include/strikeengine/simulation/WorldState.hpp",
    "include/strikeengine/systems/",
    "include/strikeengine/systems/common/",
    "include/strikeengine/systems/common/SystemUtils.hpp",

    "include/strikeengine/systems/guidance/",
    "include/strikeengine/systems/guidance/GuidanceSystem.hpp",
    "include/strikeengine/systems/physics/",
    "include/strikeengine/systems/physics/CollisionSystem.hpp",
    "include/strikeengine/systems/physics/DragSystem.hpp",
    "include/strikeengine/systems/physics/IntegrationSystem.hpp",
    "include/strikeengine/systems/physics/ThrustSystem.hpp",
    "include/strikeengine/terrain/",
    "include/strikeengine/terrain/TerrainManager.hpp",
    "include/strikeengine/utils/",
    "include/strikeengine/utils/TableLookup.hpp",
    "include/strikeengine/utils/VectorMath.hpp",
    "platform/",
    "platform/Input.hpp",
    "platform/Timer.hpp",
    "platform/Window.hpp",
    "renderer/",
    "renderer/Camera.hpp",
    "renderer/DebugDraw.hpp",
    "renderer/VulkanRenderer.cpp",
    "renderer/VulkanRenderer.hpp",
    "renderer/shaders/",
    "src/",
    "src/CMakeLists.txt",
    "src/strikeengine/",
    "src/strikeengine/guidance_laws/",
    "src/strikeengine/guidance_laws/GuidanceLaw.cpp",
    "src/strikeengine/guidance_laws/PNGuidance.cpp",
    "src/strikeengine/guidance_laws/RadarHoming.cpp",
    "src/strikeengine/io/",
    "src/strikeengine/io/ProfileLoader.cpp",
    "src/strikeengine/systems/guidance/",
    "src/strikeengine/systems/guidance/GuidanceSystem.cpp",
    "tests/",
    "tests/CMakeLists.txt",
    "tests/atmosphere_test.cpp",
    "tests/physics_test.cpp",
    "tests/profile_loader_test.cpp",
    "tests/radar_test.cpp",
    "tests/test_main.cpp",
    "tools/",
    "tools/CMakeLists.txt",
    "tools/GenerateAtmosphereTable.cpp",
    "tools/convert_srtm.cpp",
]


def create_structure():
    """Iterates through the project structure and creates missing files/dirs."""
    created_dirs = 0
    created_files = 0

    print("🚀 Starting StrikeEngine scaffolding script...")

    for path_str in PROJECT_STRUCTURE:
        path = Path(path_str)

        # Check if it's a directory (indicated by a trailing slash)
        if path_str.endswith('/'):
            if not path.exists():
                path.mkdir(parents=True, exist_ok=True)
                print(f"  [DIR]  Created: {path}")
                created_dirs += 1
        # Otherwise, it's a file
        else:
            # Ensure the parent directory exists before creating the file
            if not path.parent.exists():
                path.parent.mkdir(parents=True, exist_ok=True)
                print(f"  [DIR]  Created: {path.parent}")
                created_dirs += 1

            if not path.exists():
                path.touch()
                print(f"  [FILE] Created: {path}")
                created_files += 1

    print("\n Scaffolding complete.")
    print(f"Created {created_dirs} new directories and {created_files} new files.")
    print("Your existing files and content have not been modified.")


if __name__ == "__main__":
    create_structure()