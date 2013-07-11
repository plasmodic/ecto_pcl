cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

# Deal with catkin
find_package(catkin REQUIRED)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED sensor_msgs)
catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS sensor_msgs
)
else()
catkin_stack()
find_package(ROS REQUIRED sensor_msgs)
catkin_project(ecto_pcl
               INCLUDE_DIRS include
)
endif()

# continue with the find_package
find_package(ecto REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
)

find_package(Boost COMPONENTS signals thread)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

# install the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ecto_pcl
        DESTINATION include
        COMPONENT main
)


add_subdirectory(src)

enable_testing()
add_subdirectory(test)
add_subdirectory(samples)

# install folders
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()
