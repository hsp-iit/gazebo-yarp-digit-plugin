#include <cstdlib>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/cv/Cv.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std::chrono_literals;


int main(int argc, char** argv)
{
    /* Initialize the interpreter. */
    py::scoped_interpreter guard {};

    /**
     * Add the absolute path to the 'sensor' and 'tacto' python modules.
     * The definition 'SOURCE_PATH' is set in the CMakeLists.txt.
     */
    std::string tacto_cpp_wrapper_path = std::string(SOURCE_PATH) + "/tacto-cpp-wrapper";
    std::string tacto_path = std::string(SOURCE_PATH) + "/build/_deps/tacto-src";

    py::object sys_path_insert = py::module::import("sys").attr("path").attr("insert");
    sys_path_insert(0, tacto_cpp_wrapper_path + "/python");
    sys_path_insert(0, tacto_path);

    /**
     * Running the interpreter from here will cause sys.argv to be empty.
     * Hence, a fake entry is added given that many python modules relies on the availability of sys.argv[0].
     */
    py::module::import("sys").attr("argv").attr("insert")(0, "");

    /* Import the class from the module. */
    py::object sensor_module = py::module::import("sensor").attr("Sensor");

    /* Instantiate an object of the class. */

    py::object sensor = sensor_module
    (
        "background_path"_a = tacto_path + "/examples/conf/bg_digit_240_320.jpg",
        "configuration_path"_a = tacto_path + "/tacto/config_digit.yml"
    );

    /* Initialize the position of the sensor. */
    std::vector<double> position_vector_sensor{0.0, 0.0, 0.0};
    py::list position_sensor = py::cast(position_vector_sensor);

    /* Initialize the orientation of the sensor. */
    std::vector<double> orientation_vector_sensor{0.0, -1.5707963267948966,0.0};
    py::list orientation_sensor = py::cast(orientation_vector_sensor);

    /* Initialize the position of the object. */
    std::vector<double> position_vector_object{-0.015, 0.0, 0.048};
    py::list position_object = py::cast(position_vector_object);

    /* Initialize the orientation of the object. */
    std::vector<double> orientation_vector_object{0.0, 0.0, 0.0};
    py::list orientation_object = py::cast(orientation_vector_object);

    /* Initialize the name of the object. */
    py::str ball = "ball";

    /* Add object. */
    py::object add_object = sensor.attr("add_object");
    add_object
    (
        "mesh"_a = tacto_cpp_wrapper_path + "/mesh/textured_sphere_smooth_meters.obj",
        "object_name"_a = ball,
        "position"_a = position_object,
        "orientation"_a = orientation_object
    );

    /* Initialize YARP port. */
    Network yarp;
    BufferedPort<ImageOf<PixelRgb>> port;
    port.open("/gazebo-yarp-digit-plugin-old/output:o");

    /* Initialize the output image. */
    cv::Mat img;

    /* Initiliaze variable for simulation purpose. */
    double sinusoidal = 0;

    while (true)
    {
        /* Update position of the object for simulaiton purpose. */
        sinusoidal += 0.2;
        position_vector_object[2] += sin(sinusoidal) / 400;
        position_object = py::cast(position_vector_object);

        /* Get the image. */
        py::array_t<uint8_t> rgb = sensor.attr("render")\
        (
            "object_position"_a = position_object,
            "object_orientation"_a = orientation_object,
            "sensor_position"_a = position_sensor,
            "sensor_orientation"_a = orientation_sensor,
            "force"_a = 10.0
        );

        /* Convert the image. */
        img = cv::Mat(rgb.shape(0), rgb.shape(1), CV_8UC3, (unsigned char*)rgb.data());

        /* Prepare the output and convert fromCVMat. */
        ImageOf<PixelRgb>& output = port.prepare();
        output = yarp::cv::fromCvMat<PixelRgb>(img);

        /* Write into the port. */
        port.write();

        /* Limit update rate. */
        std::this_thread::sleep_for(30ms);
    }

    return EXIT_SUCCESS;
}
