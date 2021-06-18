# gazebo-yarp-digit-plugin
A tentative C++ wrapper for the Python based Digit tactile sensor simulation.

This branch is related to the simplest wrapper possible. Features:
- No connection to a physical environment
- A simple python mid level called "sensor_pybind.py" which connect the "renderer.py" and the "main.cpp" wrapper
- Interaction between the sensor and a single object
- Hard wired position in a known configuration

The sensor_pybind.py path must be added to the PYTHOPATH environment variable. Run:
```
export PYTHONPATH=$HOME/directory_of_gazebo-yarp-digit-plugin/python_files
```
The following sections explain how to install the dependencies.
## Pybind11
To install pybind11, the following command are required:
```
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build
cmake ../
make install
```
## TACTO
The TACTO repository must be contained in the same folder as the gazebo-yarp-digit-plugin one. The following command are required:
```
git clone https://github.com/facebookresearch/tacto.git
cd tacto
pip install -e .
pip install -r requirements/examples.txt
```
The last command installs the necessary requirements for running the TACTO examples, some of them are not strictly necessary.


## Launch example
The code simulates the contact between the sensor and a small ball which touches the sensor periodically.
Launch the YARP server
```
yarpserver
```
Open another terminal in the gazebo-yarp-digit-plugin directory, create a build directory, install and launch the binary:
```
mkdir build
cd build
cmake ../
make
./bin/wrapper
```
Then, in another terminal, launch yarpview:
```
yarpview
```
Open another terminal and connect the yarp port of the sensor to yarpview:
```
yarp connect /gazebo-yarp-digit-plugin/output:o /yarpview/img:i
```
The sinusoidal movement of the ball should now be visible in the YARP port.
