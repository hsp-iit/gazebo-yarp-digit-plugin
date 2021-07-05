#ifndef RENDERER_PLUGIN_HH
#define  RENDERER_PLUGIN_HH

#include <cmath>

#include <chrono>
#include <cstdlib>
#include <functional>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include "gazebo/sensors/sensors.hh"

#include <ignition/math/Vector3.hh>

#include <mutex>
#include <opencv2/opencv.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <thread>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/cv/Cv.h>
using namespace std::chrono_literals;
namespace gazebo
{
    class RendererPlugin;

}

class gazebo::RendererPlugin : public ModelPlugin
{
    public:

        RendererPlugin();

        ~RendererPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void UpdatePosition();
        void CallTheRenderer();
    private:

        /* Pointer to the models. */
        physics::ModelPtr sensor_model_;
        physics::ModelPtr ball_model_;

        /* Pointer to the world. */
        physics::WorldPtr world_ptr;

        /* Pointer to the connection. */
        event::ConnectionPtr updateConnection_;

        /* Pointer to the SDF value. */
        sdf::ElementPtr sdf_;

        /* Pointer to the sensor. */
        gazebo::sensors::ContactSensorPtr sensor_;

        /* Storage for the forces. */
        float forces_;

        /* Storage for the pose of the sensor. */
        ignition::math::Pose3d pose_sensor_;

        /* Semaphor that controls reading and writing of forces and pose. */
        std::mutex semaphor_;
};
#endif /* RENDERER_PLUGIN_HH */
