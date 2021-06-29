cmake_minimum_required( VERSION 3.12 FATAL_ERROR)
project(digit)
# Install models
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION share/gazebo)

# Install worlds
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds DESTINATION share/gazebo)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(RendererDigitPlugin SHARED RendererDigitPlugin.cpp)
target_link_libraries(RendererDigitPlugin ${GAZEBO_LIBRARIES})

add_library(ControlDigitPlugin SHARED ControlDigitPlugin.cpp)
target_link_libraries(ControlDigitPlugin ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
