# gazebo-yarp-digit-plugin
A tentative C++ wrapper for the Python based Digit tactile sensor simulation

The folder must be contained in the same folder of the tacto folder

This working version of the tacto wrapper has these features:
- No connection to a physical environment
- A simple python mid level called "sensor_pybind.py" which, links the "renderer.py" file of Tacto and the "main.cpp" file which uses pybind11
- Interaction between the sensor and a single object, even if the structure allows multiple objects
- Hard wired position in a known configuration (for simulation purpose, the object has a sinusoidal movement)

