cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(object_segmentation)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${pcl_definitions})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_subdirectory(euclidean_clustering)
add_subdirectory(planar_segmentation)
add_subdirectory(region_growing_clustering)
add_subdirectory(supervoxel_clustering)
