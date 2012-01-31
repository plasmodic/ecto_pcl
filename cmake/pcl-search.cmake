message(STATUS "+ Looking for PCL 1.4.0...")
find_package(PCL 1.4.0 QUIET)
if (PCL_FOUND)
  message(STATUS "+ Found PCL Version ${PCL_VERSION} with config from ${PCL_CONFIG}. ")
  add_definitions(-DPCL_VERSION_GE_140=1)
endif()

set(CMAKE_MODULE_PATH ${ecto_pcl_SOURCE_DIR}/cmake)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
