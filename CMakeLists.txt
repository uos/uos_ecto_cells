cmake_minimum_required(VERSION 2.8.3)
project(uos_ecto_cells)

find_package(catkin REQUIRED COMPONENTS
	ecto
	ecto_ros
	ecto_pcl
	pcl_ros
	tf2_ros)

find_package(PCL "1.8" REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ecto_raw_clusters
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_compile_options("-std=c++11")

ectomodule(uos_ecto_cells DESTINATION ${PROJECT_NAME} INSTALL
	src/module.cpp
	src/CloudReframer.cpp
	src/ConvexHull2Table.cpp
	src/Clusters2RecognizedObjectsArray.cpp
	src/ExtractLargestClusterIndices.cpp
	src/ConvexHull.cpp
	src/ProjectPlaneInliersPerspectively.cpp
)

pubsub_gen_wrap(object_recognition_msgs DESTINATION ${PROJECT_NAME} INSTALL)

link_ecto(uos_ecto_cells ${catkin_LIBRARIES} ${PCL_LIBRARIES})
