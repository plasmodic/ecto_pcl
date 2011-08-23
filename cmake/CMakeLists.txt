#NON INSTALL cmake configuration
if (NOT ECTO_PCL_STANDALONE)
  set(ECTO_PCL_INCLUDE_DIRS ${ecto_pcl_SOURCE_DIR}/include ${pcl_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS})
else ()
  set(ECTO_PCL_INCLUDE_DIRS ${ecto_pcl_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
endif()
configure_file(ecto_pclConfig.cmake.in ${CMAKE_BINARY_DIR}/ecto_pclConfig.cmake @ONLY)
configure_file(ecto_pclConfig-version.cmake.in ${CMAKE_BINARY_DIR}/ecto_pclConfig-version.cmake @ONLY)

#INSTALL cmake configuration
set(ECTO_PCL_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${ECTO_PCL_INCLUDE_PREFIX})
configure_file(ecto_pclConfig.cmake.in ${CMAKE_BINARY_DIR}/unix_install/ecto_pclConfig.cmake @ONLY)
configure_file(ecto_pclConfig-version.cmake.in ${CMAKE_BINARY_DIR}/unix_install/ecto_pclConfig-version.cmake @ONLY)

INSTALL(FILES
  ${CMAKE_BINARY_DIR}/unix_install/ecto_pclConfig-version.cmake
  ${CMAKE_BINARY_DIR}/unix_install/ecto_pclConfig.cmake
  DESTINATION ${ECTO_PCL_SHARE_PREFIX}
  COMPONENT main
  )
