#ifndef _CONTROL_DIGIT_PLUGIN_HH_
#define  _CONTROL_DIGIT_PLUGIN_HH_

#include <cmath>
#include <chrono>

#include <functional>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/Vector3.hh>
#include <thread>
using namespace std::chrono_literals;
namespace gazebo
{
    class ControlPlugin;

}

class gazebo::ControlPlugin : public ModelPlugin
{
    public:

        ControlPlugin();

        ~ControlPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void UpdatePosition();

    private:

        /* Pointer to the model. */
        physics::ModelPtr model_;

        /* Pointer to the connection. */
        event::ConnectionPtr updateConnection_;

        /* Time and position variable to control the sinusoidal movement of the senso. */
        double time_ = 0;
        double position_ = 0;

        /* Flag to handle the first cycle. */
        bool flag_ = false;

        /* Store for the time. */
        std::chrono::time_point<std::chrono::steady_clock> lastTime_ ;


};
#endif
