cmake_minimum_required(VERSION 3.16)

# Install worlds
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/digit_and_ball.world
              ${CMAKE_CURRENT_SOURCE_DIR}/hand_one_sensor.world
              ${CMAKE_CURRENT_SOURCE_DIR}/hand_three_sensors.world DESTINATION share/gazebo)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
