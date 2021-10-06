cmake_minimum_required( VERSION 3.16)

# Install models
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/digit
                    ${CMAKE_CURRENT_SOURCE_DIR}/left_hand_mk3_one_sensor
                    ${CMAKE_CURRENT_SOURCE_DIR}/left_hand_mk3_three_sensors
                    ${CMAKE_CURRENT_SOURCE_DIR}/mustard_bottle
                    ${CMAKE_CURRENT_SOURCE_DIR}/env_table
                DESTINATION share/gazebo/models)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
