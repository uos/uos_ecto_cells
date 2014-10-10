cmake_minimum_required(VERSION 2.8.3)
project(my_ecto_cells)

find_package(catkin REQUIRED COMPONENTS
	ecto
	ecto_ros
	pcl_ros
	tf2_ros)

find_package(PCL REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ecto_raw_clusters
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -O0")

ectomodule(my_ecto_cells DESTINATION ${PROJECT_NAME} INSTALL
	src/module.cpp
	src/CloudReframer.cpp
	src/Clusters2RecognizedObjectsArray.cpp)

pubsub_gen_wrap(object_recognition_msgs DESTINATION ${PROJECT_NAME} INSTALL)

link_ecto(my_ecto_cells ${catkin_LIBRARIES})
