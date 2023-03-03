cmake_minimum_required(VERSION 3.16)

project(gazebo-yarp-digit-plugin)

set(CMAKE_CXX_STANDARD 11)

include(FetchContent)

FetchContent_Declare(
        tacto
        GIT_REPOSITORY https://github.com/gabrielecaddeo/tacto.git
        GIT_TAG        ff447b99da8a69a332b1da90c08e97a5a1c832fe
        )
message("-- Downloading tacto repository...")
FetchContent_MakeAvailable(tacto)

add_subdirectory(tacto-cpp-wrapper)
add_subdirectory(ControlDigitPlugin)
add_subdirectory(ControlPlugin)
add_subdirectory(RendererDigitPlugin)
add_subdirectory(models)
add_subdirectory(worlds)
add_subdirectory(GraspExample)
