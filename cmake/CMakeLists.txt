#NON INSTALL cmake configuration
if (NOT ECTO_PCL_STANDALONE)
  set(ECTO_PCL_INCLUDE_DIRS ${ecto_pcl_SOURCE_DIR}/include ${pcl_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
else ()
  set(ECTO_PCL_INCLUDE_DIRS ${ecto_pcl_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
endif()
configure_file(ecto_pclConfig.cmake.in ${CMAKE_BINARY_DIR}/ecto_pclConfig.cmake @ONLY)

#INSTALL cmake configuration
configure_file(ecto_pclConfig.cmake.in ${CMAKE_BINARY_DIR}/unix_install/ecto_pclConfig.cmake @ONLY)

INSTALL(FILES
  ${CMAKE_BINARY_DIR}/unix_install/ecto_pclConfig.cmake
  DESTINATION share
  COMPONENT main
  )
