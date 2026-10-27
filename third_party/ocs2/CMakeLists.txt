# Subset of OCS2 (BSD-3, see LICENCE.txt) needed by the receding-horizon IK controller, copied from the sources of
# mindon_robotics/src/third_party/ocs2. The catkin build files are replaced by this flat CMake file.

set(OCS2_COMPILE_OPTIONS -Wno-ignored-attributes -Wno-deprecated-declarations -Wno-unused-parameter)
set(OCS2_COMPILE_DEFINITIONS
  BOOST_ALL_DYN_LINK
  PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

# Adds the library `name` from every source under `name/src`.
function(add_ocs2_library name)
  file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/${name}/src/*.cpp)
  add_library(${name} SHARED ${sources})
  target_include_directories(${name} SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${name}/include)
  target_compile_options(${name} PRIVATE ${OCS2_COMPILE_OPTIONS})
  target_compile_definitions(${name} PUBLIC ${OCS2_COMPILE_DEFINITIONS})
  target_link_libraries(${name} PUBLIC ${ARGN})
endfunction()

add_library(ocs2_thirdparty INTERFACE)
target_include_directories(ocs2_thirdparty SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/ocs2_thirdparty/include)

add_ocs2_library(ocs2_core
  ocs2_thirdparty Eigen3::Eigen Boost::system Boost::filesystem Boost::log_setup Boost::log
  OpenMP::OpenMP_CXX Threads::Threads ${CMAKE_DL_LIBS})
add_ocs2_library(ocs2_oc ocs2_core)
add_ocs2_library(ocs2_mpc ocs2_oc)
add_ocs2_library(ocs2_ddp ocs2_mpc)
add_ocs2_library(ocs2_robotic_tools ocs2_oc)
add_ocs2_library(ocs2_pinocchio_interface ocs2_robotic_tools pinocchio::pinocchio ${urdfdom_LIBRARIES})
target_include_directories(ocs2_pinocchio_interface SYSTEM PUBLIC ${urdfdom_INCLUDE_DIRS})
add_ocs2_library(ocs2_self_collision ocs2_pinocchio_interface pinocchio::pinocchio_collision hpp-fcl::hpp-fcl)
add_ocs2_library(ocs2_mobile_manipulator ocs2_ddp ocs2_self_collision)

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ocs2_mobile_manipulator)
configure_file(ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/package_path.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/ocs2_mobile_manipulator/package_path.h @ONLY)
target_include_directories(ocs2_mobile_manipulator SYSTEM PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
