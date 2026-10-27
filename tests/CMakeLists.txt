function(add_benchmark_test name)
  add_executable(${name} ${name}.cc)
  target_link_libraries(${name} PRIVATE GTest::gtest GTest::gtest_main ${ARGN})
  target_compile_definitions(${name} PRIVATE
    BENCHMARK_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_benchmark_test(test_timeline benchmark_replay)
add_benchmark_test(test_safety benchmark_replay)
add_benchmark_test(test_robot_ini_check benchmark_arm)
add_benchmark_test(test_mpc_smoke benchmark_mpc benchmark_replay)
