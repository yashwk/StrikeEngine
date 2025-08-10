add_executable(StrikeSim
        main.cpp
        App.hpp
        ScenarioLoader.hpp
)

target_link_libraries(StrikeSim PRIVATE strikeengine)

set_target_properties(StrikeSim PROPERTIES FOLDER "Applications")
