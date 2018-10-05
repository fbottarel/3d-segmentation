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
                    pcl
                    idl_tools
)

find_package(OpenCV REQUIRED)

find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

icubcontrib_set_default_prefix()

include_directories(${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS}
)

yarp_add_idl(IDL_GEN_FILES src/idl.thrift)

add_executable(${CMAKE_PROJECT_NAME} 
                    src/main.cpp
                    src/idl.thrift
                    ${IDL_GEN_FILES}
)

target_link_libraries(${CMAKE_PROJECT_NAME}
                            ${PCL_LIBRARIES}
                            ${YARP_LIBRARIES}
                            ${OpenCV_LIBS}
)

install(TARGETS ${CMAKE_PROJECT_NAME} DESTINATION bin)

icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
file(GLOB configs ${CMAKE_SOURCE_DIR}/config/*.ini)

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR}/applications)
yarp_install(FILES ${configs} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/tabletopSegmentation)


