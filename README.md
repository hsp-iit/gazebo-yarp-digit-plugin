# gazebo-yarp-digit-plugin (WIP)

A tentative C++-based Gazebo-YARP plugin for the python-based DIGIT tactile sensor simulator.

This is a work in progress and very likely to change often:
- No connections to a physical environment are considered
- The interaction between the sensor and a single object is considered

## Dependencies

- [pybind11](https://github.com/pybind/pybind11)
> can be installed, e.g. on Ubuntu, using `apt install pybind11-dev`
- [TACTO](https://github.com/facebookresearch/tacto.git)
> it is **not** required to download it as the provided `CMakeLists.txt` take care of it
- [Gazebo-YARP plugins](https://github.com/robotology/gazebo-yarp-plugins)
> To install it, refer to this [quick start guide](http://robotology.github.io/gazebo-yarp-plugins/master/)

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

## How to attach a DIGIT to your model
The following snippet code is an example on how to add the DIGIT link to your robot
```xml
<link name = "your_link_name">
    <pose>"your_pose"</pose>
    <inertial>
        <pose>0 0 0.015  0 0 0</pose>
        <mass>.1</mass>
        <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
        </inertia>
    </inertial>
    <collision name= "your_collision_name">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
            <mesh><uri> "path to the digit.STL file into the models/digit directory"</uri></mesh>
        </geometry>
     </collision>
     <visual name="your_visual_name">
        <geometry>
            <mesh><uri>"path to the digit.STL file into the models directory "</uri></mesh>
        </geometry>
    </visual>
    <sensor name='your_sensor_name' type='contact'>
        <contact>
            <collision> "your_collision_name" </collision>
        </contact>
        <update_rate> 30 </update_rate>
    </sensor>
</link>

```
The following snippet shows how to declare the RendererDigitPlugin
```xml
<plugin name="RendererDigitPlugin" filename="libRendererDigitPlugin.so">
    <ObjectName>your_object</ObjectName>
    <ObjectMeshAbsolutePath>the relative path of your object mesh</ObjectMeshAbsolutePath>
    <NumberOfSensors>N(number of sensors used)</NumberOfSensors>
    <LinkName1>base</LinkName1>
    <SensorName1>contact_digit</SensorName1>
    ...
    <LinkNameN>base</LinkNameN>
    <SensorNameN>contact_digit</SensorNameN>
    <UpdatePeriod>s</UpdatePeriod>
</plugin>
```
The `ObjectName` parameter is the name of the object the sensors are sensitive to.  
The `ObjectMeshAbsolutePath` parameter is the path of the mesh of the object.  
The `NumberOfSensors` parameter indicates the number of sensors in the model.  
The `LinkName` and `SensorName` parameters indicates the name of the links and sensors attached to them.  
The `UpdatePeriod` parameters indicates the sleep time of the rendering thread in seconds.

At the moment, the urdf snippet code is not available.

## How to run the examples

### DIGIT and ball
The code simulates the contact between the sensor and a small ball which touches the sensor periodically.

1. Run the `yarpserver`

2. Run the executable
```
gazebo ../world/digit_and_ball.world
```

3. Open a terminals and write `yarpview --name /in:i`

4. In another terminal, connect ports
```
yarp connect /gazebo-yarp-digit-plugin/output:o /in:i
```

### Left hand mk3 with one sensor

1. Run the `yarpserver`

2. Run the executable
```
gazebo world/hand_one_sensor.world
```

3. Open a terminals and write `yarpview --name /medium:i`

4. In another terminal, connect ports
```
yarp connect /gazebo-yarp-digit-plugin-sensor_link_medium"/output:o /medium:i
```
5. Launch the binary `./build/bin/grasp-example`

6. In another terminal, launch the script `GraspExample/script.sh`



### Left hand mk3 with 3 sensors
1. Run the `yarpserver`

2. Run the executable
```
gazebo world/hand_three_sensors.world
```
3. Open 3 terminals and write `yarpview --name /thumb:i` `yarpview --name /index:i` `yarpview --name /medium:i`
4. In another terminal, connect ports
```
yarp connect /gazebo-yarp-digit-plugin-sensor_link_thumb/output:o /in1:i
yarp connect /gazebo-yarp-digit-plugin-sensor_link_index/output:o /in2:i
yarp connect /gazebo-yarp-digit-plugin-sensor_link_medium/output:o /in3:i
```
