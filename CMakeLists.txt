cmake_minimum_required(VERSION 2.8)
project(ecto_pcl)

# Deal with catkin
find_package(catkin REQUIRED)
catkin_stack()
catkin_project(ecto_pcl
               INCLUDE_DIRS include
#               DEPENDS PCL # TODO fix when find_pcl_package is not called anymore
)

catkin_add_env_hooks(10.ecto_pcl SHELLS all)

# continue with the find_package
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

# deal with Python
set(python_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_PACKAGES_PATH}/ecto_pcl)

set_ecto_install_package_name(ecto_opencv)

# configure and install Python
configure_file(python/__init__.py.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
               @ONLY
)

install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
        DESTINATION ${ecto_module_PYTHON_INSTALL} COMPONENT main
)
