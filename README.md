# gazebo-yarp-digit-plugin (WIP)

A tentative C++-based Gazebo-YARP plugin for the python-based DIGIT tactile sensor simulator.

This is a work in progress and very likely to change often:
- No connections to a physical environment are considered
- The interaction between the sensor and a single object is considered
- The relative pose between the sensor and the object is fixed
- The force exchanged between the sensor and the object is fixed

## Dependencies

- [pybind11](https://github.com/pybind/pybind11)
> can be installed, e.g. on Ubuntu, using `apt install pybind11-dev`
- [TACTO](https://github.com/facebookresearch/tacto.git)
> it is **not** required to download it as the provided `CMakeLists.txt` take care of it


## How to build

**We strongly advise to create a Python virtual environment before proceeding**.

```
git clone https://github.com/robotology-playground/gazebo-yarp-digit-plugin.git
cd gazebo-yarp-digit-plugin
mkdir build
cd build
cmake ../
```
> At this point, please wait for the `TACTO` repository to be cloned.

```
pip install -r _deps/tacto-src/requirements/requirements.txt
make -j
```


## How to run the example
The code simulates the contact between the sensor and a small ball which touches the sensor periodically.

1. Run the `yarpserver`

2. Run the executable

```
cd gazebo-yarp-digit-plugin
./build/bin/tacto-cpp-wrapper
```

3. Open a `yarpview --name /in:i`

4. Connect ports
```
yarp connect /gazebo-yarp-digit-plugin/output:o /in:i
```
