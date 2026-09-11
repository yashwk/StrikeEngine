# W1/W2/W3 fidelity-spine validation (deep session DoD)
add_executable(rigidbody_test validation/rigidbody_test.cpp)
target_link_libraries(rigidbody_test PRIVATE strikeengine)

add_executable(vehicleconfig_test validation/vehicleconfig_test.cpp)
target_link_libraries(vehicleconfig_test PRIVATE strikeengine)

add_executable(intercept_test validation/intercept_test.cpp)
target_link_libraries(intercept_test PRIVATE strikeengine)

add_executable(rocket_mvp_test validation/rocket_mvp_test.cpp)
target_link_libraries(rocket_mvp_test PRIVATE strikeengine)

add_executable(coefficient_table_test validation/coefficient_table_test.cpp)
target_link_libraries(coefficient_table_test PRIVATE strikeengine)

add_executable(rocket_mvp_tables_test validation/rocket_mvp_tables_test.cpp)
target_link_libraries(rocket_mvp_tables_test PRIVATE strikeengine)

add_executable(fins_test validation/fins_test.cpp)
target_link_libraries(fins_test PRIVATE strikeengine)

add_executable(airframe_test validation/airframe_test.cpp)
target_link_libraries(airframe_test PRIVATE strikeengine)

add_executable(integrator_test validation/integrator_test.cpp)
target_link_libraries(integrator_test PRIVATE strikeengine)

add_executable(seeker_test validation/seeker_test.cpp)
target_link_libraries(seeker_test PRIVATE strikeengine)
target_compile_definitions(seeker_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(seeker_rich_test validation/seeker_rich_test.cpp)
target_link_libraries(seeker_rich_test PRIVATE strikeengine)
target_compile_definitions(seeker_rich_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(seeker_fidelity_test validation/seeker_fidelity_test.cpp)
target_link_libraries(seeker_fidelity_test PRIVATE strikeengine)
target_compile_definitions(seeker_fidelity_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(track_manager_fidelity_test validation/track_manager_fidelity_test.cpp)
target_link_libraries(track_manager_fidelity_test PRIVATE strikeengine)

add_executable(guidance_fidelity_test validation/guidance_fidelity_test.cpp)
target_link_libraries(guidance_fidelity_test PRIVATE strikeengine)

add_executable(navigation_test validation/navigation_test.cpp)
target_link_libraries(navigation_test PRIVATE strikeengine)

add_executable(lever_arm_test validation/lever_arm_test.cpp)
target_link_libraries(lever_arm_test PRIVATE strikeengine)

add_executable(earth_rate_gyro_test validation/earth_rate_gyro_test.cpp)
target_link_libraries(earth_rate_gyro_test PRIVATE strikeengine)

add_executable(coning_sculling_test validation/coning_sculling_test.cpp)
target_link_libraries(coning_sculling_test PRIVATE strikeengine)

add_executable(environment_test validation/environment_test.cpp)
target_link_libraries(environment_test PRIVATE strikeengine)

add_executable(earth_test validation/earth_test.cpp)
target_link_libraries(earth_test PRIVATE strikeengine)

add_executable(earth_frames_test validation/earth_frames_test.cpp)
target_link_libraries(earth_frames_test PRIVATE strikeengine)

add_executable(earth_transport_test validation/earth_transport_test.cpp)
target_link_libraries(earth_transport_test PRIVATE strikeengine)

add_executable(spherical_gravity_test validation/spherical_gravity_test.cpp)
target_link_libraries(spherical_gravity_test PRIVATE strikeengine)

add_executable(earth_fixed_test validation/earth_fixed_test.cpp)
target_link_libraries(earth_fixed_test PRIVATE strikeengine)

add_executable(ecef_kernel_test validation/ecef_kernel_test.cpp)
target_link_libraries(ecef_kernel_test PRIVATE strikeengine)

add_executable(guidance_test validation/guidance_test.cpp)
target_link_libraries(guidance_test PRIVATE strikeengine)

add_executable(seeker_intercept_test validation/seeker_intercept_test.cpp)
target_link_libraries(seeker_intercept_test PRIVATE strikeengine)
target_compile_definitions(seeker_intercept_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(track_manager_test validation/track_manager_test.cpp)
target_link_libraries(track_manager_test PRIVATE strikeengine)

add_executable(trajectory_test validation/trajectory_test.cpp)
target_link_libraries(trajectory_test PRIVATE strikeengine)

add_executable(scenario_test validation/scenario_test.cpp)
target_link_libraries(scenario_test PRIVATE strikeengine)

add_executable(reporting_test validation/reporting_test.cpp)
target_link_libraries(reporting_test PRIVATE strikeengine)

add_executable(kernel_lifecycle_test validation/kernel_lifecycle_test.cpp)
target_link_libraries(kernel_lifecycle_test PRIVATE strikeengine)

add_executable(failure_test validation/failure_test.cpp)
target_link_libraries(failure_test PRIVATE strikeengine)

add_executable(propulsion_test validation/propulsion_test.cpp)
target_link_libraries(propulsion_test PRIVATE strikeengine)

add_executable(global_terrain_test validation/global_terrain_test.cpp)
target_link_libraries(global_terrain_test PRIVATE strikeengine)
target_compile_definitions(global_terrain_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
if(STRIKEENGINE_WITH_GDAL)
    target_compile_definitions(global_terrain_test PRIVATE STRIKEENGINE_WITH_GDAL=1)
endif()

add_executable(config_wiring_test validation/config_wiring_test.cpp)
target_link_libraries(config_wiring_test PRIVATE strikeengine)

add_executable(staging_warhead_test validation/staging_warhead_test.cpp)
target_link_libraries(staging_warhead_test PRIVATE strikeengine)

add_executable(warhead_falloff_test validation/warhead_falloff_test.cpp)
target_link_libraries(warhead_falloff_test PRIVATE strikeengine)

add_executable(serialization_test validation/serialization_test.cpp)
target_link_libraries(serialization_test PRIVATE strikeengine)

add_executable(profile_database_test validation/profile_database_test.cpp)
target_link_libraries(profile_database_test PRIVATE strikeengine)
target_compile_definitions(profile_database_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(designer_pipeline_test validation/designer_pipeline_test.cpp)
target_link_libraries(designer_pipeline_test PRIVATE strikeengine)
target_compile_definitions(designer_pipeline_test PRIVATE
    STRIKEENGINE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(multithread_test validation/multithread_test.cpp)
target_link_libraries(multithread_test PRIVATE strikeengine)

add_test(NAME rigidbody COMMAND rigidbody_test)
set_tests_properties(rigidbody PROPERTIES TIMEOUT 120)

add_test(NAME vehicleconfig COMMAND vehicleconfig_test)
set_tests_properties(vehicleconfig PROPERTIES TIMEOUT 120)

add_test(NAME intercept COMMAND intercept_test)
set_tests_properties(intercept PROPERTIES TIMEOUT 120)

add_test(NAME rocket_mvp COMMAND rocket_mvp_test)
set_tests_properties(rocket_mvp PROPERTIES TIMEOUT 120)

add_test(NAME coefficient_table COMMAND coefficient_table_test)
set_tests_properties(coefficient_table PROPERTIES TIMEOUT 120)

add_test(NAME rocket_mvp_tables COMMAND rocket_mvp_tables_test)
set_tests_properties(rocket_mvp_tables PROPERTIES TIMEOUT 120)

add_test(NAME fins COMMAND fins_test)
set_tests_properties(fins PROPERTIES TIMEOUT 120)

add_test(NAME airframe COMMAND airframe_test)
set_tests_properties(airframe PROPERTIES TIMEOUT 120)

add_test(NAME integrator COMMAND integrator_test)
set_tests_properties(integrator PROPERTIES TIMEOUT 120)

add_test(NAME seeker COMMAND seeker_test)
set_tests_properties(seeker PROPERTIES TIMEOUT 120)

add_test(NAME seeker_rich COMMAND seeker_rich_test)
add_test(NAME seeker_fidelity COMMAND seeker_fidelity_test)
set_tests_properties(seeker_rich PROPERTIES TIMEOUT 120)

add_test(NAME navigation COMMAND navigation_test)
add_test(NAME navigation_aiding COMMAND navigation_aiding_test)
set_tests_properties(navigation_aiding PROPERTIES TIMEOUT 120)
set_tests_properties(navigation PROPERTIES TIMEOUT 120)

add_test(NAME lever_arm COMMAND lever_arm_test)
set_tests_properties(lever_arm PROPERTIES TIMEOUT 120)

add_test(NAME earth_rate_gyro COMMAND earth_rate_gyro_test)
set_tests_properties(earth_rate_gyro PROPERTIES TIMEOUT 120)

add_test(NAME coning_sculling COMMAND coning_sculling_test)
set_tests_properties(coning_sculling PROPERTIES TIMEOUT 120)

add_test(NAME environment COMMAND environment_test)
set_tests_properties(environment PROPERTIES TIMEOUT 120)

add_test(NAME earth COMMAND earth_test)
set_tests_properties(earth PROPERTIES TIMEOUT 120)

add_test(NAME earth_frames COMMAND earth_frames_test)
set_tests_properties(earth_frames PROPERTIES TIMEOUT 120)

add_test(NAME earth_transport COMMAND earth_transport_test)
set_tests_properties(earth_transport PROPERTIES TIMEOUT 120)

add_test(NAME spherical_gravity COMMAND spherical_gravity_test)
set_tests_properties(spherical_gravity PROPERTIES TIMEOUT 120)

add_test(NAME earth_fixed COMMAND earth_fixed_test)
set_tests_properties(earth_fixed PROPERTIES TIMEOUT 120)

add_test(NAME ecef_kernel COMMAND ecef_kernel_test)
set_tests_properties(ecef_kernel PROPERTIES TIMEOUT 120)

add_test(NAME guidance COMMAND guidance_test)
set_tests_properties(guidance PROPERTIES TIMEOUT 120)

add_test(NAME seeker_intercept COMMAND seeker_intercept_test)
set_tests_properties(seeker_intercept PROPERTIES TIMEOUT 120)

add_test(NAME track_manager COMMAND track_manager_test)
add_test(NAME track_manager_fidelity COMMAND track_manager_fidelity_test)
add_test(NAME guidance_fidelity COMMAND guidance_fidelity_test)
set_tests_properties(track_manager PROPERTIES TIMEOUT 120)

add_test(NAME trajectory COMMAND trajectory_test)
set_tests_properties(trajectory PROPERTIES TIMEOUT 120)

add_test(NAME scenario COMMAND scenario_test)
set_tests_properties(scenario PROPERTIES TIMEOUT 120)

add_test(NAME reporting COMMAND reporting_test)
set_tests_properties(reporting PROPERTIES TIMEOUT 120)

add_test(NAME kernel_lifecycle COMMAND kernel_lifecycle_test)
set_tests_properties(kernel_lifecycle PROPERTIES TIMEOUT 120)

add_test(NAME failure COMMAND failure_test)
set_tests_properties(failure PROPERTIES TIMEOUT 120)

add_test(NAME propulsion COMMAND propulsion_test)
set_tests_properties(propulsion PROPERTIES TIMEOUT 120)

add_test(NAME global_terrain COMMAND global_terrain_test)
set_tests_properties(global_terrain PROPERTIES TIMEOUT 120)

add_test(NAME config_wiring COMMAND config_wiring_test)
set_tests_properties(config_wiring PROPERTIES TIMEOUT 120)

add_test(NAME staging_warhead COMMAND staging_warhead_test)
set_tests_properties(staging_warhead PROPERTIES TIMEOUT 120)

add_test(NAME warhead_falloff COMMAND warhead_falloff_test)
set_tests_properties(warhead_falloff PROPERTIES TIMEOUT 120)

add_test(NAME serialization COMMAND serialization_test)
set_tests_properties(serialization PROPERTIES TIMEOUT 120)

add_test(NAME profile_database COMMAND profile_database_test)
set_tests_properties(profile_database PROPERTIES TIMEOUT 120)

add_test(NAME designer_pipeline COMMAND designer_pipeline_test)
set_tests_properties(designer_pipeline PROPERTIES TIMEOUT 120)

add_test(NAME multithread COMMAND multithread_test)
set_tests_properties(multithread PROPERTIES TIMEOUT 120)

add_executable(navigation_aiding_test validation/navigation_aiding_test.cpp)
target_link_libraries(navigation_aiding_test PRIVATE strikeengine)
