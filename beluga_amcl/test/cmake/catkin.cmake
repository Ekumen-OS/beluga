find_package(rostest REQUIRED)

catkin_add_gmock(test_amcl_node_utils test_amcl_node_utils.cpp)
target_link_libraries(test_amcl_node_utils ${PROJECT_NAME} ${catkin_LIBRARIES} gmock_main)

add_rostest_gtest(test_amcl_nodelet amcl_nodelet.test test_amcl_nodelet.cpp)
target_link_libraries(test_amcl_nodelet ${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} gtest_main)
