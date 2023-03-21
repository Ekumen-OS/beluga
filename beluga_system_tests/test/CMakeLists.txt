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
        GTest::gtest_main
        rosbag2_cpp::rosbag2_cpp
)

include(GoogleTest)
gtest_discover_tests(test_system WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
