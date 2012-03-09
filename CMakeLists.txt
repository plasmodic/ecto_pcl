cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

find_package(catkin REQUIRED)
catkin_stack()
catkin_project(ecto_pcl
               INCLUDE_DIRS include
#               DEPENDS PCL # TODO fix when find_pcl_package is not called anymore
)

find_package(ecto REQUIRED)

rosbuild_lite_init()

find_pcl_package()

include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
)

find_package(Boost COMPONENTS signals thread)

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

include_directories(include)
add_subdirectory(src)

enable_testing()
add_subdirectory(test)
add_subdirectory(samples)
