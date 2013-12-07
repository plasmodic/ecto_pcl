macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(pcl_${pyfile}
    ${CATKIN_DEVEL_PREFIX}/env.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

pytest(test_import)

