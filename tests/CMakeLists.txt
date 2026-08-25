add_executable(run_tests validation/singlerun_test.cpp)
target_link_libraries(run_tests PRIVATE 
    strikeengine
    Vulkan::Vulkan
)

add_executable(sweep_test validation/sweep_test.cpp)
target_link_libraries(sweep_test PRIVATE strikeengine)

add_executable(mc_test validation/mc_test.cpp)
target_link_libraries(mc_test PRIVATE strikeengine)

add_executable(optimizer_test optimizer_test.cpp)
target_link_libraries(optimizer_test PRIVATE strikeengine)

add_test(NAME StrikeEngineTests COMMAND run_tests)

set_target_properties(run_tests PROPERTIES FOLDER "Tests")
