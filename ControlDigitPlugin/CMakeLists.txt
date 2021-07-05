cmake_minimum_required(VERSION 3.16)

find_package(gazebo REQUIRED)

find_package(pybind11 CONFIG)

find_package(OpenCV REQUIRED)

find_package(YARP REQUIRED COMPONENTS os sig cv)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
add_library(ControlDigitPlugin SHARED src/ControlDigitPlugin.cpp)

target_include_directories(ControlDigitPlugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(ControlDigitPlugin PRIVATE
                                                pybind11::embed
                                                YARP::YARP_cv
                                                YARP::YARP_init
                                                YARP::YARP_os
                                                YARP::YARP_sig
                                                ${OpenCV_LIBS}
                                                ${GAZEBO_LIBRARIES})
