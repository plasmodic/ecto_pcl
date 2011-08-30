macro(validate_sample pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(validate_pcl_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/validate_sample.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

validate_sample(colorize_clusters)
validate_sample(openni_viewer)
validate_sample(pcd_rw)
validate_sample(ros_sample)
validate_sample(tabletop_segmentation)

