/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H


class TrajectoryGenerator
{
public:

    TrajectoryGenerator(ignition::math::Pose3<double> start_pose);

    ~TrajectoryGenerator();

    void SetNewPose(ignition::math::Pose3<double> new_pose, double trajectory_duration, std::chrono::time_point<std::chrono::steady_clock> start_time);

    void SetNewPose(ignition::math::Pose3<double> new_pose, ignition::math::Pose3<double> starting_pose, double trajectory_duration, std::chrono::time_point<std::chrono::steady_clock> start_time);

    bool UpdatePose(std::chrono::time_point<std::chrono::steady_clock> time);

    double GetLinearVelocityX();
    double GetLinearVelocityY();
    double GetLinearVelocityZ();

    ignition::math::Pose3<double> GetPose();

private:

    ignition::math::Quaternion<double> ComputeOrientation();

    double ComputePosition(double starting_coordinate, double final_coordinate);

    double ComputeVelocity(double starting_coordinate, double final_coordinate);

    ignition::math::Pose3<double> starting_pose_, current_pose_, final_pose_;

    ignition::math::Vector3<double> current_velocity_;

    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    double trajectory_duration_ = 0;

    double current_time_ = 0;
};
#endif /* TRAJECTORY_GENERATOR_H */
