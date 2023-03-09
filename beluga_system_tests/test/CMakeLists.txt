find_package(rosbag2_cpp REQUIRED)
find_package(beluga REQUIRED)
find_package(beluga_amcl REQUIRED)
find_package(GTest REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(test_system test_system.cpp)

target_link_libraries(test_system
    PUBLIC
        beluga::beluga
        beluga_amcl::amcl_node_utils
        GTest::Main  # deprecated since cmake 3.20 but compatible with minimum 3.16
        rosbag2_cpp::rosbag2_cpp
)

include(GoogleTest)
gtest_add_tests(test_system "" AUTO WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# gtest_discover_tests(test_system WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
