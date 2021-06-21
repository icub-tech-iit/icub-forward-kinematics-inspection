################################################################################
#                                                                              #
# Copyright (C) 2021 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)

project(fkin)

find_package(ICUBcontrib REQUIRED)
find_package(YARP REQUIRED COMPONENTS os sig math)
find_package(ICUB REQUIRED COMPONENTS ctrlLib iKin)
find_package(iDynTree REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

add_executable(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_sig
                                              YARP::YARP_math
                                              ICUB::ctrlLib
                                              ICUB::iKin
                                              ${iDynTree_LIBRARIES})

target_sources(${PROJECT_NAME} PRIVATE src/main.cpp 
                                       src/fkinModule.cpp 
                                       src/fkinModule.h)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB urdf ${CMAKE_CURRENT_SOURCE_DIR}/model/model.urdf)

yarp_install(FILES ${urdf} DESTINATION ${ICUBCONTRIB_ROBOTS_INSTALL_DIR}/icubGazeboSim)

icubcontrib_add_uninstall_target()
