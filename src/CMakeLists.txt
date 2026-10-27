add_library(benchmark_mpc SHARED
  mpc/mpc_config.cc
  mpc/ocs2_interface_builder.cc
  mpc/ocs2_ik_controller.cc
  mpc/dual_arm_ocs2_ik_controller.cc
)
target_include_directories(benchmark_mpc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(benchmark_mpc PUBLIC ocs2_mobile_manipulator glog::glog yaml-cpp Eigen3::Eigen Threads::Threads)

add_library(benchmark_arm SHARED
  arm/fake_arm.cc
  arm/marvin_arm.cc
  arm/robot_ini_check.cc
)
target_include_directories(benchmark_arm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(benchmark_arm PUBLIC marvin_sdk glog::glog Eigen3::Eigen Threads::Threads)

add_library(benchmark_replay SHARED
  replay/config.cc
  replay/recorder.cc
  replay/safety.cc
  replay/timeline.cc
  replay/trajectory.cc
)
target_include_directories(benchmark_replay PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(benchmark_replay PUBLIC benchmark_arm yaml-cpp Eigen3::Eigen)

add_executable(replay replay/replay_main.cc)
target_link_libraries(replay PRIVATE benchmark_replay benchmark_mpc benchmark_arm ${urdfdom_LIBRARIES})
target_include_directories(replay SYSTEM PRIVATE ${urdfdom_INCLUDE_DIRS})
