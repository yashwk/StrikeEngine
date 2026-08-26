add_executable(run_tests validation/singlerun_test.cpp)
target_link_libraries(run_tests PRIVATE strikeengine)

add_executable(sweep_test validation/sweep_test.cpp)
target_link_libraries(sweep_test PRIVATE strikeengine)

add_executable(mc_test validation/mc_test.cpp)
target_link_libraries(mc_test PRIVATE strikeengine)

add_executable(optimizer_test optimizer_test.cpp)
target_link_libraries(optimizer_test PRIVATE strikeengine)

# W1/W2/W3 fidelity-spine validation (deep session DoD)
add_executable(rigidbody_test validation/rigidbody_test.cpp)
target_link_libraries(rigidbody_test PRIVATE strikeengine)

add_executable(vehicleconfig_test validation/vehicleconfig_test.cpp)
target_link_libraries(vehicleconfig_test PRIVATE strikeengine)

add_executable(intercept_test validation/intercept_test.cpp)
target_link_libraries(intercept_test PRIVATE strikeengine)

add_executable(integrator_test validation/integrator_test.cpp)
target_link_libraries(integrator_test PRIVATE strikeengine)

add_executable(seeker_test validation/seeker_test.cpp)
target_link_libraries(seeker_test PRIVATE strikeengine)
target_compile_definitions(seeker_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(navigation_test validation/navigation_test.cpp)
target_link_libraries(navigation_test PRIVATE strikeengine)

add_executable(environment_test validation/environment_test.cpp)
target_link_libraries(environment_test PRIVATE strikeengine)

add_executable(guidance_test validation/guidance_test.cpp)
target_link_libraries(guidance_test PRIVATE strikeengine)

add_executable(scenario_test validation/scenario_test.cpp)
target_link_libraries(scenario_test PRIVATE strikeengine)

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

add_test(NAME rigidbody COMMAND rigidbody_test)
set_tests_properties(rigidbody PROPERTIES TIMEOUT 120)

add_test(NAME vehicleconfig COMMAND vehicleconfig_test)
set_tests_properties(vehicleconfig PROPERTIES TIMEOUT 120)

add_test(NAME intercept COMMAND intercept_test)
set_tests_properties(intercept PROPERTIES TIMEOUT 120)

add_test(NAME integrator COMMAND integrator_test)
set_tests_properties(integrator PROPERTIES TIMEOUT 120)

add_test(NAME seeker COMMAND seeker_test)
set_tests_properties(seeker PROPERTIES TIMEOUT 120)

add_test(NAME navigation COMMAND navigation_test)
set_tests_properties(navigation PROPERTIES TIMEOUT 120)

add_test(NAME environment COMMAND environment_test)
set_tests_properties(environment PROPERTIES TIMEOUT 120)

add_test(NAME guidance COMMAND guidance_test)
set_tests_properties(guidance PROPERTIES TIMEOUT 120)

add_test(NAME scenario COMMAND scenario_test)
set_tests_properties(scenario PROPERTIES TIMEOUT 120)

set_target_properties(run_tests PROPERTIES FOLDER "Tests")
