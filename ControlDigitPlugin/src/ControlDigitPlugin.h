/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CONTROL_DIGIT_PLUGIN_H
#define CONTROL_DIGIT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class ControlPlugin;
}

class gazebo::ControlPlugin : public ModelPlugin
{
    public:

        ControlPlugin();

        ~ControlPlugin();

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

        void UpdatePosition();

    private:

        /* Methods to compute the ideal position and velocity of the trajectory. */
        double ComputePosition(double starting_coordinate, double final_coordinate);
        double ComputeVelocity(double starting_coordinate, double final_coordinate, double elapsed);

        /* Pointer to the model. */
        physics::ModelPtr model_;

        /* Pointer to the connection. */
        event::ConnectionPtr updateConnection_;

        /* Time and position variable to control the sinusoidal movement of the senso. */
        double time_ = 0;
        double position_ = 0;

        /* Flag to handle the first cycle. */
        bool is_first_time_ = true;

        /* Store for the time. */
        std::chrono::time_point<std::chrono::steady_clock> last_time_ ;


        ignition::math::Pose3<double> starting_pose_;
};
#endif /* CONTROL_DIGIT_PLUGIN_H */
