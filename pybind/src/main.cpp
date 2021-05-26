#include <cstdlib>
#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include <pybind11/embed.h>
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
    
    //initialize the interpreter
    py::scoped_interpreter guard {};
    
    //
    py::object sensor_module= py::module_::import("sensor_pybind").attr("Sensor");
    py::object sensor=sensor_module();
    
    py::object add_object= sensor.attr("add_object");
    
    py::list position;
    position.append(-0.015);
    position.append(0.0);
    position.append(0.048);
    
    py::list orientation;
    orientation.append(0.0);
    orientation.append(0.0);
    orientation.append(0.0);
    py::print(position);
    add_object("mesh"_a= "../../tacto/examples/objects/textured_sphere_smooth_meters.obj", "obj_id"_a=2, "position"_a=position, "orientation"_a= orientation);
    Network yarp;
    BufferedPort<ImageOf<PixelRgb> > port;
    port.open("/sparrow");
    cv::Mat img;
    /*cv::Mat test = cv::imread("ciao.png", cv::IMREAD_COLOR);
    yarp::sig::ImageOf<yarp::sig::PixelRgb> img_yarp = yarp::cv::fromCvMat<PixelRgb>(test);
    std::cout << test.cols << " " << test.rows << std::endl;
    std::cout << img_yarp.width() << " " << img_yarp.height() << std::endl;

    std::cout << test.at<cv::Vec3b>(10, 10) << std::endl;
    auto pixel = img_yarp.pixel(10, 10);
    std::cout << (int) pixel.r << " " << (int)pixel.g << " " << (int)pixel.b << std::endl;*/
    while (true)
    {
        //sensor.attr("get_image")();ls

        py::array_t<uint8_t> rgb= sensor.attr("get_image")();
        img = cv::Mat(rgb.shape(0), rgb.shape(1), CV_8UC3, (unsigned char*)rgb.data());
            
        //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

        //exit(0); 
        
        
        //yarp::cv::toCvMat(output)=img;
        ImageOf<PixelRgb>& output = port.prepare();
        output=yarp::cv::fromCvMat<PixelRgb>(img);  
        //std::cout << output.width() << " " << output.height() << std::endl;
        //auto pixel = output.pixel(10, 10);
        //std::cout << (int) pixel.r << " " << (int)pixel.g << " " << (int)pixel.b << std::endl;
        port.write();        
        
        std::this_thread::sleep_for(300ms);
    }
    return EXIT_SUCCESS;

}
