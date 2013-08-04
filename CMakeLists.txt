cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

# Deal with catkin
find_package(catkin REQUIRED)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto sensor_msgs)
catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS pcl_ros sensor_msgs
)
else()
catkin_stack()
find_package(ROS REQUIRED ecto sensor_msgs)
catkin_project(ecto_pcl
               INCLUDE_DIRS include
)
set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/${PROJECT_NAME})
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
endif()

# continue with the find_package
find_package(PCL REQUIRED)
include_directories(SYSTEM
                    ${PCL_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Boost COMPONENTS signals thread)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

# install the include directory
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ecto_pcl
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
)


add_subdirectory(src)

enable_testing()
add_subdirectory(test)
add_subdirectory(samples)
