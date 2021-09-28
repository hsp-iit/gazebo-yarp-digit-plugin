cmake_minimum_required(VERSION 3.16)

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

add_library(ControlDigitPlugin SHARED src/ControlDigitPlugin.cpp)

target_include_directories(ControlDigitPlugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(ControlDigitPlugin PRIVATE ${GAZEBO_LIBRARIES})
