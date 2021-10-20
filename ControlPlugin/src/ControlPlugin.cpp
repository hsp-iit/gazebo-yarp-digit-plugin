/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include "ControlPlugin.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::ControlPlugin)

gazebo::ControlPlugin::ControlPlugin()
{}


gazebo::ControlPlugin::~ControlPlugin()
{}


template<class T>
bool gazebo::ControlPlugin::LoadParameterFromSDF(sdf::ElementPtr sdf, const std::string& name, T& value)
{
    /* Check if the element exists. */
    if (!(sdf->HasElement(name)))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", parameter " + name + " does not exist. Returning false.";

        return false;
    }

    /* Get the associated parameter. */
    sdf::ParamPtr parameter = sdf->GetElement(name)->GetValue();

    /* Check if the value can be intrepreted as a T. */
    if (!parameter->Get<T>(value))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", parameter " + name + " is not valid. Returning false.";

        return false;
    }

    return true;
}


void gazebo::ControlPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    /* Store the pointer of the model. */
    object_model_ = model;


    /*Load the Link parameter. */
    if (!LoadParameterFromSDF(sdf, "Link", link_name_))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", error with Link parameter. Closing the plugin thread.";

        std::terminate();
    }

    /* Store the starting pose of the base link of the object. */
    start_pose_object_ = object_model_->GetLink(link_name_)->WorldPose();

    /* Initialize the trajectory generator. */
    trajectory_generator_ = std::make_unique<TrajectoryGenerator>(start_pose_object_);


    /* Initialize the trajectory generator with the starting pose. */
    trajectory_generator_->SetNewPose(ignition::math::Pose3<double>(
                                      ignition::math::Vector3<double>(start_pose_object_.X(), start_pose_object_.Y(), start_pose_object_.Z()),
                                      start_pose_object_.Rot()),
                                      trajectory_duration_,
                                      std::chrono::steady_clock::now());

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/control-object-port/rpc:i"))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot open rpc port. Closing the plugin thread.";

        std::terminate();
    }

    if (!(yarp().attachAsServer(port_rpc_)))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot attach RPC port to the respond handler. Closing the plugin thread.";

        std::terminate();
    }

    /* Load the gain parameter. */
    std::string gain;

    if (!LoadParameterFromSDF(sdf, "Gain", gain))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", error with Gain parameter. Closing the plugin thread.";

        std::terminate();
    }

    try
    {
        p_gain_ = stoi(gain);
    }
    catch (const std::invalid_argument& ia)
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", invalid argument: " << ia.what() << ". Closing the plugin thread.";

        std::terminate();
    }

    std::string port_name;

    if (!LoadParameterFromSDF(sdf, "PortOutputPoseName", port_name))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", error with Gain parameter. Closing the plugin thread.";

        std::terminate();
    }

    /* Open the output port to send out the position of the object. */
    port_pose_.open(port_name);

    /* Update the posiiton by calling the UpdatePosition method. */
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo::ControlPlugin::UpdateControl, this));
}


std::string gazebo::ControlPlugin::NewPose(const double x, const double y, const double z, const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration)
{
    std::string message_to_user;

    mutex_.lock();

    /* Check if the previous movement is finished. */
    if (is_motion_done_)
    {
        /* Update the pose and the trajectory duration. */
        trajectory_generator_->SetNewPose(ignition::math::Pose3<double>(
                                          ignition::math::Vector3<double>(x, y, z),
                                          ignition::math::Quaternion<double>(ignition::math::Vector3<double>(axis_x, axis_y, axis_z), angle)),
                                          duration,
                                          std::chrono::steady_clock::now());

        message_to_user = "Command accepted";
    }
    else
        message_to_user = "Command not accepted";

    mutex_.unlock();

    return message_to_user;
}


std::string gazebo::ControlPlugin::NewOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration)
{
    std::string message_to_user;

    mutex_.lock();

    /* Check if the previous movement is finished. */
    if (is_motion_done_)
    {
        /* Update the pose and the trajectory duration. */
        ignition::math::Pose3<double> current_pose = object_model_->GetLink(link_name_)->WorldPose();

        /* Update the pose and the trajectory duration. */
        trajectory_generator_->SetNewPose(ignition::math::Pose3<double>(
                                          ignition::math::Vector3<double>(current_pose.X(), current_pose.Y(), current_pose.Z()),
                                          ignition::math::Quaternion<double>(ignition::math::Vector3<double>(axis_x, axis_y, axis_z), angle)),
                                          duration,
                                          std::chrono::steady_clock::now());

        message_to_user = "Command accepted";
    }
    else
        message_to_user = "Command not accepted";

    mutex_.unlock();

    return message_to_user;
}


std::string gazebo::ControlPlugin::NewRelativePosition(const double x, const double y, const double z, const double duration)
{
    std::string message_to_user;

    mutex_.lock();

    /* Check if the previous movement is finished. */
    if (is_motion_done_)
    {
        /* Update the positiion and the trajectory duration. */
        ignition::math::Pose3<double> current_pose = object_model_->GetLink(link_name_)->WorldPose();

        /* Update the pose and the trajectory duration. */
        trajectory_generator_->SetNewPose(ignition::math::Pose3<double>(
                                          ignition::math::Vector3<double>(current_pose.X() + x, current_pose.Y() + y, current_pose.Z() + z),
                                          current_pose.Rot()),
                                          duration,
                                          std::chrono::steady_clock::now());

        message_to_user =  "Command accepted";
    }
    else
        message_to_user = "Command not accepted";

    mutex_.unlock();

    return message_to_user;
}


std::string gazebo::ControlPlugin::NewRelativeOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const std::string& fixed_axes, const double duration)
{
    std::string message_to_user;
    bool set_new_pose = false;
    ignition::math::Quaternion<double> rotation;

    mutex_.lock();

    /* Check if the previous movement is finished. */
    if (is_motion_done_)
    {
        /* Update the positiion and the trajectory duration. */
        ignition::math::Pose3<double> current_pose = object_model_->GetLink(link_name_)->WorldPose();

        /* Check wether to pre-multiply or post-multiply the given transformation. */
        if (fixed_axes == "true" || fixed_axes == "True")
        {
            rotation = current_pose.Rot() * ignition::math::Quaternion<double>(ignition::math::Vector3<double>(axis_x, axis_y, axis_z), angle);
            set_new_pose = true;

            message_to_user = "Command accepted";
        }
        else if (fixed_axes == "false" || fixed_axes == "False")
        {
            rotation = ignition::math::Quaternion<double>(ignition::math::Vector3<double>(axis_x, axis_y, axis_z), angle) * current_pose.Rot();
            set_new_pose = true;

            message_to_user =  "Command accepted";
        }
        else
            message_to_user = "Command not accepted. Insert a correct string for the fixed_axes parameter";

        if (set_new_pose)
        {
            /* Update the pose and the trajectory duration. */
            trajectory_generator_->SetNewPose(ignition::math::Pose3<double>(
                                              ignition::math::Vector3<double>(current_pose.X(), current_pose.Y(), current_pose.Z()),
                                              rotation),
                                              duration,
                                              std::chrono::steady_clock::now());
        }
    }
    else
        message_to_user = "Command not accepted. The motion is not finished yet.";

    mutex_.unlock();

    return message_to_user;
}


std::string gazebo::ControlPlugin::GoHome()
{
    std::string message_to_user;

    mutex_.lock();

    /* Check if the previous movement is finished. */
    if (is_motion_done_)
    {
        /* Go back to the starting configuration. */
        /* Update the pose and the trajectory duration. */
        trajectory_generator_->SetNewPose(start_pose_object_, 10, std::chrono::steady_clock::now());

        message_to_user = "Command accepted";
    }
    else
        message_to_user = "Command not accepted";

    mutex_.unlock();

    return message_to_user;
}


void gazebo::ControlPlugin::UpdateControl()
{
    mutex_.lock();

    is_motion_done_ = trajectory_generator_->UpdatePose(std::chrono::steady_clock::now());

    mutex_.unlock();

    /* Get the desired pose. */
    ignition::math::Pose3<double> desired_pose = trajectory_generator_->GetPose();
    ignition::math::Matrix3<double> matrix_desired (desired_pose.Rot());

    /* Get the actual pose. */
    ignition::math::Pose3<double> pose = object_model_->GetLink(link_name_)->WorldPose();
    ignition::math::Matrix3<double> matrix_actual (pose.Rot());

    /* Define the axis-angle variables and the gain of the controller. */
    ignition::math::Vector3<double> axis, velocity_vector, velocity_vector_angular;
    double angle;

    /* Calculate the values to be passed to the controller. */
    ignition::math::Quaternion<double> log_quaternion (matrix_actual.Transposed() * matrix_desired);
    log_quaternion.ToAxis(axis, angle);
    axis = matrix_actual * axis * angle;

    /* Retrieve the linear and angular velocity. */
    velocity_vector = matrix_actual * object_model_->GetLink(link_name_)->RelativeLinearVel();
    velocity_vector_angular = matrix_actual * object_model_->GetLink(link_name_)->RelativeAngularVel();

    /* Apply the forces to the object. */
    double controlX = p_gain_ * (desired_pose.X() - pose.X()) + 2 * std::sqrt(p_gain_) * (trajectory_generator_->GetLinearVelocityX() - velocity_vector.X());
    double controlY = p_gain_ * (desired_pose.Y() - pose.Y()) + 2 * std::sqrt(p_gain_) * (trajectory_generator_->GetLinearVelocityY() - velocity_vector.Y());
    double controlZ = p_gain_ * (desired_pose.Z() - pose.Z()) + 2 * std::sqrt(p_gain_) * (trajectory_generator_->GetLinearVelocityZ() - velocity_vector.Z());

    object_model_->GetLink(link_name_)->SetForce(ignition::math::Vector3d(controlX, controlY, controlZ));

    /* Apply the torque to the body. */
    object_model_->GetLink(link_name_)->SetTorque(p_gain_ * axis + 2 * std::sqrt(p_gain_) * (0 - velocity_vector_angular));

    ignition::math::Vector3<double> actual_axis;
    double actual_angle;
    pose.Rot().ToAxis(actual_axis, actual_angle);

    yarp::sig::VectorOf<double> & actual_pose = port_pose_.prepare();
    
    actual_pose.resize(7);
    actual_pose[0] = pose.X();
    actual_pose[1] = pose.Y();
    actual_pose[2] = pose.Z();
    actual_pose[3] = actual_axis[0];
    actual_pose[4] = actual_axis[1];
    actual_pose[5] = actual_axis[2];
    actual_pose[6] = actual_angle;

    port_pose_.write();
}


bool gazebo::ControlPlugin::Close()
{
    port_rpc_.close();

    return true;
}
