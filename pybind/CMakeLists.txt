cmake_minimum_required(VERSION 3.15)

project(wrapper)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
# mandatory use of these packages
find_package(YARP 3.3.0 REQUIRED)
find_package(ICUBcontrib REQUIRED)

find_package(pybind11 CONFIG)
find_package(OpenCV REQUIRED)
# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()


add_executable(wrapper src/main.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

target_link_libraries(wrapper PRIVATE ${OpenCV_LIBS} pybind11::embed ${YARP_LIBRARIES})

install(TARGETS wrapper DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
