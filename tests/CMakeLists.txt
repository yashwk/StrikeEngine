add_executable(run_tests validation/singlerun_test.cpp)
target_link_libraries(run_tests PRIVATE strikeengine)

add_executable(sweep_test validation/sweep_test.cpp)
target_link_libraries(sweep_test PRIVATE strikeengine)

add_executable(mc_test validation/mc_test.cpp)
target_link_libraries(mc_test PRIVATE strikeengine)

add_executable(optimizer_test optimizer_test.cpp)
target_link_libraries(optimizer_test PRIVATE strikeengine)

# Full-simulation validation programs. They run scenarios end-to-end, so they
# need generous timeouts; they also write CSV outputs to the build dir.
add_test(NAME singlerun COMMAND run_tests)
set_tests_properties(singlerun PROPERTIES TIMEOUT 300)

add_test(NAME sweep COMMAND sweep_test)
set_tests_properties(sweep PROPERTIES TIMEOUT 300)

add_test(NAME montecarlo COMMAND mc_test)
set_tests_properties(montecarlo PROPERTIES TIMEOUT 600)

add_test(NAME optimizer COMMAND optimizer_test)
set_tests_properties(optimizer PROPERTIES TIMEOUT 600)

set_target_properties(run_tests PROPERTIES FOLDER "Tests")
