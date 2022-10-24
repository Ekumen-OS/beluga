add_library(core INTERFACE)
target_include_directories(core INTERFACE include)
target_compile_features(core INTERFACE cxx_std_17)

add_executable(core_test)
target_sources(core_test PRIVATE test/particle_filter_test.cpp)
target_link_libraries(core_test PRIVATE gmock_main core)
target_compile_options(core_test PRIVATE -Wall -Werror)

include(GoogleTest)
gtest_discover_tests(core_test)
