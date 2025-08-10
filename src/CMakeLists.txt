file(GLOB_RECURSE STRIKEENGINE_SOURCES "strikeengine/**/*.cpp")

add_library(strikeengine STATIC ${STRIKEENGINE_SOURCES})

target_compile_definitions(strikeengine PUBLIC GLM_ENABLE_EXPERIMENTAL)

target_include_directories(strikeengine PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)

target_link_libraries(strikeengine PUBLIC
        glm
        nlohmann_json
)

message(STATUS "Configured core library: strikeengine")
