add_executable(pcl_node pcl_node.cpp)
target_link_libraries(pcl_node
    ${sensor_msgs_LIBRARIES}
    ${pcl_LIBRARIES}
    )
add_executable(pcl_cloud_wr pcl_cloud_wr.cpp)
target_link_libraries(pcl_cloud_wr
    ${pcl_LIBRARIES}
    )

add_executable(pcl_nop pcl_nop.cpp)
target_link_libraries(pcl_nop
    ${pcl_LIBRARIES}
    )