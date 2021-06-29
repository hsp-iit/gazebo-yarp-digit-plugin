#ifndef _RENDERER_DIGIT_PLUGIN_HH_
#define  _RENDERER_DIGIT_PLUGIN_HH_

#include <cmath>
#include <chrono>

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

namespace gazebo
{
    class RendererDigitPlugin;

}

class gazebo::RendererDigitPlugin : public ModelPlugin
{
    public:

        RendererDigitPlugin();

        ~RendererDigitPlugin();

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

        void UpdatePosition();

    private:

        /* Pointer to the model. */
        physics::ModelPtr model;

        /* Pointer to the connection. */
        event::ConnectionPtr updateConnection;

        /* Pointer to the SDF value. */
        sdf::ElementPtr sdf;

        /* PID controller. */

        double time = 0;

        double position = 0;


        /* Pointer to the sensor. */
        gazebo::sensors::ContactSensorPtr sensor;

        /* Flag to handle the first cycle. */
        bool flag = false;

        /* Store for the time. */
        std::chrono::time_point<std::chrono::steady_clock> lastTime ;


};
#endif
