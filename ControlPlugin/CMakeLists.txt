#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.16)

set(TARGET_NAME set-new-pose)

find_package(gazebo REQUIRED)

find_package(YARP REQUIRED COMPONENTS sig os idl_tools)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(${TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)
yarp_add_idl(${TARGET_NAME}_THRIFT_SRC ${${TARGET_NAME}_THRIFT_HDR})

add_library(ControlPlugin SHARED src/ControlPlugin.cpp  ${${TARGET_NAME}_THRIFT_SRC})

target_include_directories(ControlPlugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(ControlPlugin PRIVATE YARP::YARP_sig YARP::YARP_os YARP::YARP_init ${GAZEBO_LIBRARIES})
