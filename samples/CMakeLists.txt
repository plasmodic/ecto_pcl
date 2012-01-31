macro(validate_sample pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(validate_pcl_${pyfile}
    ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()


macro(data_download PATH_VAR DATA_FILE)
  set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
  set(${PATH_VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
  if(NOT EXISTS ${${PATH_VAR}})
    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${PATH_VAR}}")
    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${PATH_VAR}})
  endif()
endmacro()

validate_sample(pcd_rw)

macro(desktop_test_sample pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(desktop_test_pcl_${pyfile}
    ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

if(DESKTOP_TEST)
  data_download(CB_PCD cb.pcd)
  data_download(STILL_LIFE still_life.pcd)

  #opens display
  validate_sample(pcd_viewer)

  desktop_test_sample(pcd_rw ${CB_PCD})
  desktop_test_sample(pcd_viewer ${STILL_LIFE})
  desktop_test_sample(colorize_clusters)
  desktop_test_sample(openni_viewer)
  desktop_test_sample(tabletop_segmentation)

endif()

if (ROS_TEST)
  validate_sample(ros_sample)
endif()
