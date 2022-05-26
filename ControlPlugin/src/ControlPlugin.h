
/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CONTROL_PLUGIN_H
#define CONTROL_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mutex>

#include <thrift/SetNewPoseIDL.h>

#include "TrajectoryGenerator.cpp"

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

namespace gazebo
{
    class ControlPlugin;
}


class gazebo::ControlPlugin : public ModelPlugin,
                                  public SetNewPoseIDL
{
    public:

        ControlPlugin();

        ~ControlPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void UpdateControl();

        bool Close();

        /*
        * IDL interface.
        */

        std::string NewPose(const double x, const double y, const double z, const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration) override;

        std::string NewOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration) override;

        std::string NewPosition(const double x, const double y, const double z, const double duration) override;

        std::string NewRelativePosition(const double x, const double y, const double z, const double duration) override;

        std::string NewRelativeOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const std::string& fixed_axes, const double duration) override;

        std::string GoHome() override;
        
        std::string SetControlStatus(const std::string& status);

    private:

        template<class T>
        bool LoadParameterFromSDF(sdf::ElementPtr sdf, const std::string &name, T& value);

        /* Pointer to the model. */
        physics::ModelPtr object_model_;

        /* Pointer to the connection. */
        event::ConnectionPtr updateConnection_;

        /* Time to complete the movement. */
        double trajectory_duration_ = 5;

        /* Store for the starting pose. */
        ignition::math::Pose3<double> start_pose_object_;

        /* Flag to handle the first cycle. */
        bool is_motion_done_ = false;
        
        /* Flag to handle control status */
        bool is_control_on_ = true;

        /* Pointer to the trajectory generator. */
        std::unique_ptr<TrajectoryGenerator> trajectory_generator_;

        /* RPC port. */
        yarp::os::Port port_rpc_;

        /* Gain of the controller. */
        double p_gain_;

        std::mutex mutex_;

        /* Port to send out the real-time pose. */
        yarp::os::BufferedPort<yarp::sig::Vector> port_pose_;

        std::string link_name_;

        yarp::os::Network yarp_network_;

};
#endif /* CONTROL_PLUGIN_H */
