add_executable(MissionCLI main.cpp)

target_link_libraries(MissionCLI PRIVATE strikeengine)

set_target_properties(MissionCLI PROPERTIES FOLDER "Applications")
