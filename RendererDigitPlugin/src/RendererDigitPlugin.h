/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef RENDERER_PLUGIN_H
#define RENDERER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include "gazebo/sensors/sensors.hh"

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
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

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

        void UpdateStates();

        void RenderingThread();

    private:

        template<class T>
        bool LoadParameterFromSDF(sdf::ElementPtr sdf, const std::string &name, T& value);

        /* Method and its output varibale to compute forces. */
        std::unordered_map<std::string, float> ComputeForces();

        /* Pointer to the models. */
        physics::ModelPtr sensor_model_;
        physics::ModelPtr object_model_;

        /* Pointer to the connection. */
        event::ConnectionPtr updateConnection_;

        /* Pointer to the sensor. */
        std::unordered_map<std::string, gazebo::sensors::ContactSensorPtr> sensors_;

        /* Storage for the forces. */
        std::unordered_map<std::string, float> forces_;

        /* Storage for the pose of the sensor and the object. */
        std::unordered_map<std::string, ignition::math::Pose3d> pose_sensors_;
        ignition::math::Pose3d pose_object_;

        /* Semaphor that controls reading and writing of forces and pose. */
        std::mutex mutex_;

        std::vector<std::string> sensors_link_name_;

        double update_period_;

        std::string object_mesh_absolute_path_, object_name_;
};
#endif /* RENDERER_PLUGIN_H */
