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

set_target_properties(run_tests PROPERTIES FOLDER "Tests")
