# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)

project(fkin)

find_package(ICUBcontrib REQUIRED)
find_package(YARP REQUIRED COMPONENTS os sig math)
find_package(ICUB REQUIRED COMPONENTS ctrlLib iKin)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()

add_executable(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
                                              YARP::YARP_sig
                                              YARP::YARP_math
                                              ICUB::ctrlLib
                                              ICUB::iKin)

target_sources(${PROJECT_NAME} PRIVATE src/main.cpp 
                                       src/fkinModule.cpp 
                                       src/fkinModule.h)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

icubcontrib_add_uninstall_target()
