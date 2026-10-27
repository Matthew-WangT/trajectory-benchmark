# Tianji Marvin control SDK (contrlSDK100343, Apache-2.0), built from source with the flags of its makefile.
# It requires a controller firmware of 100343 or newer.
file(GLOB marvin_sdk_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(marvin_sdk SHARED ${marvin_sdk_sources})
target_include_directories(marvin_sdk SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_definitions(marvin_sdk PUBLIC CMPL_LIN)
target_compile_options(marvin_sdk PRIVATE -w)
target_link_libraries(marvin_sdk PUBLIC Threads::Threads rt)
