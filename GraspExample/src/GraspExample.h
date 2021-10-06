 /*
  * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
  *
  * This software may be modified and distributed under the terms of the
  * GPL-2+ license. See the accompanying LICENSE file for details.
  */

#ifndef GRASP_EXAMPLE_H
#define GRASP_EXAMPLE_H

#include <thrift/SetNewPoseIDL.h>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>


class GraspExample : public yarp::os::RFModule,
                     public SetNewPoseIDL
{
public:
    /*
    * RFModule interface.
    */

    bool configure(yarp::os::ResourceFinder& rf) override;

    bool close() override;

    double getPeriod() override;

    bool updateModule() override;

    /*
    * IDL interface
    */

    std::string NewPose(const double x, const double y, const double z, const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration) override;

    std::string NewOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration) override;

    std::string NewRelativePosition(const double x, const double y, const double z, const double duration) override;

    std::string NewRelativeOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration, const std::string& fixed_axes) override;

    std::string GoHome() override;

    std::string Grasp() override;

    /* State class for the state machine. */
    enum class State{Idle, GoHome, SendMessage, StartGrasp, Wait, CompleteGrasp};

private:

    bool OpenRemoteControlboard(const std::string& robot_name, const std::string& part_name);

    yarp::dev::PolyDriver driver_;
    yarp::dev::IControlMode * control_;
    yarp::dev::IPositionControl * position_;

    std::chrono::steady_clock::time_point start_time_;

    yarp::os::Port port_rpc_;
    yarp::os::RpcClient port_rpc_client_;
    yarp::os::Bottle cmd_, reply_;

    std::mutex mutex_state_, mutex_bottle_;

    /* State machine variable and methods. */
    State state_;
    GraspExample::State GetState();
    void SetState(GraspExample::State state);
};
#endif /* GRASP_EXAMPLE_H */
