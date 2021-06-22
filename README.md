# gazebo-yarp-digit-plugin
A tentative C++ wrapper for the Python based Digit tactile sensor simulation
The folder must be contained in the same folder the tacto folder is contained

This branch is related to the simplest wrapper possible. Features:
- No connection to a physical environment
- A simple python mid level called "sensor_pybind.py" which connect the "renderer.py" and the "main.cpp" wrapper
- Interaction between the sensor and a single object
- Hard wired position in a known configuration

