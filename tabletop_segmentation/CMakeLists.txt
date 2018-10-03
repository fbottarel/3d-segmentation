cmake_minimum_required(VERSION 3.0)

project(tabletop_segmentation
        LANGUAGES CXX
)

find_package(YARP REQUIRED 
                COMPONENTS
                    OS
                    dev
                    sig
                    math
                    pcl)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS}
)

add_executable(${CMAKE_PROJECT_NAME} 
                    src/main.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME}
                            ${PCL_LIBRARIES}
                            ${YARP_LIBRARIES}
                            ${OpenCV_LIBS}
)



