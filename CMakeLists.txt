cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

# Deal with catkin
find_package(catkin REQUIRED)
find_package(catkin REQUIRED ecto sensor_msgs pcl_conversions)
catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS pcl_conversions sensor_msgs
)

# continue with the find_package
find_package(PCL REQUIRED)
include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Boost COMPONENTS signals thread)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

# install the include directory
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ecto_pcl/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
)


add_subdirectory(src)

enable_testing()
add_subdirectory(test)
add_subdirectory(samples)
