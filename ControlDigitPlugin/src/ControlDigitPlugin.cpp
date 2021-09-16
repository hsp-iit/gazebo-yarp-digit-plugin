/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include "ControlDigitPlugin.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

ControlPlugin::ControlPlugin()
{}

ControlPlugin::~ControlPlugin()
{}

void ControlPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
	/* Store the pointer of the model. */
	model_ = model;

    starting_pose_ = model_->WorldPose();

    /*Update the posiiton by calling the UpdatePosition method. */
	updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ControlPlugin::UpdatePosition, this));
}

void ControlPlugin::UpdatePosition()
{
	/* Store the start time. */
	auto start = std::chrono::steady_clock::now();

	/* Check whether it is the first cycle or not. */
	if (is_first_time_ == true)
	{
		last_time_ = start;

		is_first_time_= false;
		return;
	}

	/* Get the position from gazebo. */
	ignition::math::Pose3d pose = model_->WorldPose();
    ignition::math::Matrix3<double> matrix_actual (pose.Rot());

	/* Store the time difference. */
	double elapsed = std::chrono::duration<double>(start - last_time_).count();

	/* Compute the sinusoidal movement. */
	time_ += elapsed ;
	position_ = sin(time_ * 0.3) * 0.05;

    /* Initialize the gain of the controller. */
    double p_gain = 10;

    /* Initialize the axis/angle and velocity vectors variables. */
    ignition::math::Vector3<double> axis, velocity_vector, velocity_vector_angular;
    double angle;

    /* Retrieve the velocity vectors. */
    velocity_vector = matrix_actual * model_->RelativeLinearVel();
    velocity_vector_angular = matrix_actual * model_->RelativeAngularVel();

	/* Along the x axis, the sensor has to follow the sinusoidal movement. */
	/* Along the y axis, the sensor has to stay at 0 coordinate. */
	/* Along the z axis, the sensor has to stay at 0.0048, hard wired height of the sdf file. */
	double controlX = p_gain * (position_ - pose.X()) + 2 * std::sqrt(p_gain) * (ControlPlugin::ComputeVelocity(pose.X(), position_, elapsed) - velocity_vector.X());
	double controlY = p_gain * (starting_pose_.Y()) + 2 * std::sqrt(p_gain) * (ControlPlugin::ComputeVelocity(pose.Y(), starting_pose_.Y(), elapsed) - velocity_vector.Y());
	double controlZ = p_gain * (starting_pose_.Z()) + 2 * std::sqrt(p_gain) * (ControlPlugin::ComputeVelocity(pose.Z(), starting_pose_.Z(), elapsed) - velocity_vector.Z());

    /* Apply the forces to the sensor. */
	model_->GetLink("base")->SetForce(ignition::math::Vector3d(controlX, controlY, controlZ));

    /* Compute the orientation error. */
    ignition::math::Quaternion<double>(ignition::math::Matrix3<double>(pose.Rot()).Transposed() * ignition::math::Matrix3<double>(starting_pose_.Rot())).ToAxis(axis, angle);
    axis = matrix_actual * axis * angle;

    /* Apply the torque to the sensor. */
    model_->GetLink()->SetTorque(p_gain * axis + 2*std::sqrt(p_gain) * (0 - velocity_vector_angular));

	/* Store the time for the next cycle. */
	last_time_ = start;
}

double ControlPlugin::ComputePosition(double starting_coordinate, double final_coordinate)
{
    return starting_coordinate + 10 * (final_coordinate - starting_coordinate) - 15 * (final_coordinate - starting_coordinate) + 6 * (final_coordinate - starting_coordinate);
}

double ControlPlugin::ComputeVelocity(double starting_coordinate, double final_coordinate, double elapsed)
{
    return (30*(final_coordinate - starting_coordinate) - 60 * (final_coordinate - starting_coordinate) + 30 * (final_coordinate- starting_coordinate) )/ elapsed;
}
