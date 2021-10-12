cmake_minimum_required(VERSION 3.16)

project(gazebo-yarp-digit-plugin)

set(CMAKE_CXX_STANDARD 11)

include(FetchContent)

FetchContent_Declare(
        tacto
        GIT_REPOSITORY https://github.com/facebookresearch/tacto.git
        GIT_TAG        d9b6a8bddd83af40f4e65e08da5ce55790c4f2a5
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
