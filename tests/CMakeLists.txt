file(GLOB TEST_SOURCES "*.cpp")

add_executable(run_tests ${TEST_SOURCES})

target_link_libraries(run_tests PRIVATE strikeengine)

add_test(NAME StrikeEngineTests COMMAND run_tests)

set_target_properties(run_tests PROPERTIES FOLDER "Tests")
