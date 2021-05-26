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
    
    //initialize the interpreter
    py::scoped_interpreter guard {};
    
    
    py::object sensor_module= py::module_::import("sensor_pybind").attr("Sensor");
    py::object sensor=sensor_module();
    
    py::object add_object= sensor.attr("add_object");
    //intiialize position
    //convert from std vector so that it is easiest
    std::vector<double> position_vector_ball{-0.015, 0.0, 0.048};
    py::list position= py::cast(position_vector_ball);
    
    /*
    we can use also the append method
   
    py::list position;
    position.append(-0.015);
    position.append(0.0);
    position.append(0.048);
    */

    //initialize orientation
    //convert from std vector so that it is easiest
    std::vector<double> orientation_vector_ball{0.0, 0.0, 0.0};
    py::list orientation= py::cast(orientation_vector_ball);
    
    //name of the object to add    
    py::str ball= "ball";
    //initialize the dictionaries for the python wrapper
    py::dict dict_objects_orientation, dict_objects_position;
    dict_objects_position[ball]= position;
    dict_objects_orientation[ball]= orientation;
    
    
    //add object
    add_object("mesh"_a= "../../tacto/examples/objects/textured_sphere_smooth_meters.obj", "obj_name"_a=ball, "position"_a=dict_objects_position[ball], "orientation"_a= dict_objects_orientation[ball]);
    
    //initialize YARP port
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
        //here we could update pos and orientation
        
        //position_vector_ball[2]+=0.0001;
        //position= py::cast(position_vector_ball);
        //dict_objects_position[ball]=position;
        //py::print(position);
        
        //get the image
        py::array_t<uint8_t> rgb= sensor.attr("get_image")("objects_positions"_a=dict_objects_position, "objects_orientations"_a= dict_objects_orientation);
        
        //convert the image
        img = cv::Mat(rgb.shape(0), rgb.shape(1), CV_8UC3, (unsigned char*)rgb.data());

        //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

        //exit(0); 
        
        
        //yarp::cv::toCvMat(output)=img;
        
        //prepare the output and convert fromCVMat
        ImageOf<PixelRgb>& output = port.prepare();
        output=yarp::cv::fromCvMat<PixelRgb>(img);  


        //std::cout << output.width() << " " << output.height() << std::endl;
        //auto pixel = output.pixel(10, 10);
        //std::cout << (int) pixel.r << " " << (int)pixel.g << " " << (int)pixel.b << std::endl;

        //write into the port
        port.write();        
        

        //useful just for simulation purpose
        std::this_thread::sleep_for(300ms);
    }
    return EXIT_SUCCESS;

}
