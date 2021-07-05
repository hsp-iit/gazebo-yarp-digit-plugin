cmake_minimum_required(VERSION 3.16)
set(TARGET_NAME tacto-cpp-wrapper)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

find_package(pybind11 CONFIG)

find_package(OpenCV REQUIRED)

find_package(YARP REQUIRED COMPONENTS os sig cv)

add_definitions(-DSOURCE_PATH=\"${CMAKE_SOURCE_DIR}\")

add_executable(${TARGET_NAME} src/main.cpp)

target_link_libraries(${TARGET_NAME} PRIVATE
                                     pybind11::embed
                                     YARP::YARP_cv
                                     YARP::YARP_init
                                     YARP::YARP_os
                                     YARP::YARP_sig
                                     ${OpenCV_LIBS}
)
