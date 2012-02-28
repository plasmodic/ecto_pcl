cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

find_package(ecto REQUIRED)

set(ECTO_PCL_VERSION 0.1.0)
set(ECTO_PCL_PREFIX ${PROJECT_NAME}-${ECTO_PCL_VERSION}
  CACHE INTERNAL "Mangled prefixer" FORCE)
set(ECTO_PCL_INCLUDE_PREFIX include/${ECTO_PCL_PREFIX}
  CACHE INTERNAL "Where ecto_pcl includes will live" FORCE)
set(ECTO_PCL_SHARE_PREFIX share/${ECTO_PCL_PREFIX}
  CACHE INTERNAL "Where ecto_pcl share will live" FORCE)

include(cmake/pcl-search.cmake)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

find_package(Boost COMPONENTS signals thread)

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

include_directories(include)
add_subdirectory(cmake)
add_subdirectory(src)

enable_testing()
add_subdirectory(test)
add_subdirectory(samples)

INSTALL(DIRECTORY include
        DESTINATION ${ECTO_PCL_INCLUDE_PREFIX}
        COMPONENT main
        )


