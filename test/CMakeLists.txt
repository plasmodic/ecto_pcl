macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(pcl_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

#macro(data_download PATH_VAR DATA_FILE)
#  set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
#  set(${PATH_VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
#  if(NOT EXISTS ${${PATH_VAR}})
#    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${PATH_VAR}}")
#    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${PATH_VAR}})
#  endif()
#endmacro()

add_executable(pcl_node pcl_node.cpp)
target_link_libraries(pcl_node
    ${pcl_LIBRARIES}
    ecto_ros_ectomodule
    )
add_executable(pcl_cloud_wr pcl_cloud_wr.cpp)
target_link_libraries(pcl_cloud_wr
    ${pcl_LIBRARIES}
    ecto_ros_ectomodule
    )

add_executable(pcl_nop pcl_nop.cpp)
target_link_libraries(pcl_nop
    ${pcl_LIBRARIES}
    )

pytest(test_import)
