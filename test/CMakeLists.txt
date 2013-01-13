macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
if (ROS_GROOVY_FOUND)
  add_test(pcl_${pyfile}
    ${CATKIN_DEVEL_PREFIX}/env.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
else()
  add_test(pcl_${pyfile}
    ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endif()
endmacro()

pytest(test_import)

