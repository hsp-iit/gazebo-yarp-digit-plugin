#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TARGET_NAME grasp-example)

find_package(YARP REQUIRED COMPONENTS dev os sig idl_tools)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(${TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)

yarp_add_idl(${TARGET_NAME}_THRIFT_SRC ${${TARGET_NAME}_THRIFT_HDR})

add_executable(${TARGET_NAME}
               src/GraspExample.h
               src/main.cpp
               src/GraspExample.cpp
               ${${TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${TARGET_NAME} PRIVATE include src)

target_link_libraries(${TARGET_NAME}
                      YARP::YARP_dev
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
)
