/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include "TrajectoryGenerator.h"


TrajectoryGenerator::TrajectoryGenerator(ignition::math::Pose3<double> start_pose)
{
    /* Initialize the generator so that the object stays in the actual position. */
    starting_pose_ = start_pose;
    final_pose_ = start_pose;
}


TrajectoryGenerator::~TrajectoryGenerator()
{}


void TrajectoryGenerator::SetNewPose(ignition::math::Pose3<double> new_pose, double trajectory_duration, std::chrono::time_point<std::chrono::steady_clock> start_time)
{
    /* Store the new values of the variables. */
    starting_pose_ = final_pose_;
    final_pose_ = new_pose;
    trajectory_duration_ = trajectory_duration;
    start_time_ = start_time;
}


ignition::math::Quaternion<double> TrajectoryGenerator::ComputeOrientation()
{
    /* Check if the time variable reached the limit. */
    if (current_time_ > trajectory_duration_)
    {
        current_time_ = trajectory_duration_;
    }

    return ignition::math::Quaternion<double>::Slerp(current_time_ / trajectory_duration_, starting_pose_.Rot(), final_pose_.Rot());
}


double TrajectoryGenerator::ComputePosition(double starting_coordinate, double final_coordinate)
{
    /* Check if the time variable reached the limit. */
    if (current_time_ > trajectory_duration_)
    {
        current_time_ = trajectory_duration_;
    }

    return starting_coordinate + (10*(final_coordinate - starting_coordinate) / (std::pow(trajectory_duration_, 3))) * (std::pow(current_time_, 3))
                               - (15*(final_coordinate - starting_coordinate) / (std::pow(trajectory_duration_, 4))) * (std::pow(current_time_, 4))
                               + (6*(final_coordinate - starting_coordinate) / (std::pow(trajectory_duration_, 5))) * (std::pow(current_time_, 5));
}


double TrajectoryGenerator::ComputeVelocity(double starting_coordinate, double final_coordinate)
{
    /* Check if the time variable reached the limit. */
    if (current_time_ > trajectory_duration_)
    {
        current_time_ = trajectory_duration_;
    }

    return (30*(final_coordinate - starting_coordinate) / (std::pow(trajectory_duration_, 3))) * (std::pow(current_time_ , 2))
         - (60*(final_coordinate - starting_coordinate) / (std::pow(trajectory_duration_, 4))) * (std::pow(current_time_ , 3))
         + (30*(final_coordinate - starting_coordinate) / (std::pow(trajectory_duration_, 5))) * (std::pow(current_time_ , 4));
}


bool TrajectoryGenerator::UpdatePose(std::chrono::time_point<std::chrono::steady_clock> time)
{
    /* Update the current trajectory time. */
    current_time_ = std::chrono::duration<double>(time - start_time_).count();

    /* Update the current pose. */
    current_pose_.Set(ignition::math::Vector3<double>(TrajectoryGenerator::ComputePosition(starting_pose_.X(), final_pose_.X()),
                                                      TrajectoryGenerator::ComputePosition(starting_pose_.Y(), final_pose_.Y()),
                                                      TrajectoryGenerator::ComputePosition(starting_pose_.Z(), final_pose_.Z())),
                                                      TrajectoryGenerator::ComputeOrientation());

    /* Update the current velocity. */
    current_velocity_.Set(ComputeVelocity(starting_pose_.X(), final_pose_.X()),
                          ComputeVelocity(starting_pose_.Y(), final_pose_.Y()),
                          ComputeVelocity(starting_pose_.Z(), final_pose_.Z()));

    /* Check if the time variable reached the limit. */
    if (current_time_ >= trajectory_duration_)
    {
        return true;
    }
    else
    {
        return false;
    }
}


double TrajectoryGenerator::GetLinearVelocityX()
{
    return current_velocity_.X();
}


double TrajectoryGenerator::GetLinearVelocityY()
{
    return current_velocity_.Y();
}


double TrajectoryGenerator::GetLinearVelocityZ()
{
    return current_velocity_.Z();
}


ignition::math::Pose3<double> TrajectoryGenerator::GetPose()
{
    return current_pose_;
}
