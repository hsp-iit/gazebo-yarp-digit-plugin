/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include "GraspExample.h"
#include <yarp/os/LogStream.h>

bool GraspExample::configure(yarp::os::ResourceFinder &rf)
{
    /* Get the robot name. */
    std::string robot_name = rf.check("robot_name", yarp::os::Value("SIM_LEFT_HAND")).asString();

    /* Open hand control board. */
    if(!OpenRemoteControlboard(robot_name, "left_hand"))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", error with the RemoteControlBoard. Closing the plugin thread.";

        return false;
    }

    /* Open RPC port and attach to respond handler. */
    if (!port_rpc_.open("/grasp-example/rpc:i"))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot open input RPC port. Closing the module.";

        return false;
    }

    if (!(yarp().attachAsServer(port_rpc_)))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot attach RPC port to the respond handler. Closing the module.";

        return false;
    }

    /* Open RPC client port. */
    if(!port_rpc_client_.open("/grasp-example/rpc:o"))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot open output RPC port. Closing the module.";

        return false;
    }

    /* Connect the client with the plugin's server. */
    yarp::os::Network::connect("/grasp-example/rpc:o", "/control-hand-port/rpc:i");

    /* Set the initial state. */
    GraspExample::SetState(State::Idle);

    return true;
}


bool GraspExample::OpenRemoteControlboard(const std::string& robot_name, const std::string& part_name)
{
    /* Set the property of the device */
    yarp::os::Property prop;
    prop.put("device", "remote_controlboard");
    prop.put("local", "/realsense-holder-calibration/" + part_name);
    prop.put("remote", "/" + robot_name + "/" + part_name);

    /* Try to open the driver_. */
    if(!driver_.open(prop))
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot open " + part_name + ". Returning false.";

        return false;
    }

    /* Open the views and check them. */
    bool ok = true;
    ok &= driver_.view(control_);
    ok &= driver_.view(position_);

    if (!ok)
    {
        yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot open " + part_name + " interfaces. Returning false.";

        return false;
    }

    /* Set the control mode. */
    control_->setControlMode(0, VOCAB_CM_POSITION);
    control_->setControlMode(1, VOCAB_CM_POSITION);
    control_->setControlMode(2, VOCAB_CM_POSITION);
    control_->setControlMode(3, VOCAB_CM_POSITION);
    control_->setControlMode(4, VOCAB_CM_POSITION);
    control_->setControlMode(5, VOCAB_CM_POSITION);
    control_->setControlMode(6, VOCAB_CM_POSITION);

    return true;
}


bool GraspExample::updateModule()
{
    /* Get the actual State. */
    State state = GetState();

    switch(state)
    {
        case State::GoHome:
        {
            /* Set the joints to the initial position. */
            std::vector<int> joints{0, 1, 2, 3, 4, 5, 6};
            std::vector<double> joints_hand {0, 0, 0, 0, 0, 0, 0};
            position_->positionMove(joints.size(), joints.data(), joints_hand.data());

            /* Update the bottle. */
            mutex_bottle_.lock();

            cmd_.clear();
            cmd_.addString("GoHome");

            mutex_bottle_.unlock();

            /* Update the state. */
            GraspExample::SetState(State::SendMessage);

            break;
        }

        case State::Idle:
        {
            /* Do nothing. */

            break;
        }

        case State::SendMessage:
        {
            /* Send the message to the plugin. */
            mutex_bottle_.lock();

            if(!(port_rpc_client_.write(cmd_, reply_)))
                yError() << "At line " << __LINE__ << ", in function " << __FUNCTION__ << ", cannot send the message";

            mutex_bottle_.unlock();

            /* Update the state. */
            GraspExample::SetState(State::Idle);

            break;
        }

        case State::StartGrasp:
        {
            std::vector<int> joints{0, 1, 2, 3, 4, 5, 6};

            /* Get the current time. */
            start_time_ = std::chrono::steady_clock::now();

            /* Set the values of the joints to start moving the fingers. */
            std::vector<double> joints_hand {0 ,0 ,0 ,0 ,62 ,72 ,0 };
            position_->positionMove(joints.size(), joints.data(), joints_hand.data());

            /* Update the state. */
            GraspExample::SetState(State::Wait);

            break;
        }

        case State::Wait:
        {
            /* Check if the grasp had enough time to complete the first movement. */
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time_).count() > 7)
                GraspExample::SetState(State::CompleteGrasp);

            break;
        }

        case State::CompleteGrasp:
        {
            std::vector<int> joints{0, 1, 2, 3, 4, 5, 6};

            /* Get the current time. */
            start_time_ = std::chrono::steady_clock::now();

            /* Set the values of the joints to complete the grasp. */
            std::vector<double> joints_hand {80 ,0 ,0 ,0 ,62 ,72 ,0};
            position_->positionMove(joints.size(), joints.data(), joints_hand.data());

            /* Update the state. */
            GraspExample::SetState(State::Idle);

            break;
        }
    }
    return true;
}


double GraspExample::getPeriod()
{
    return 1.0;
}

std::string GraspExample::Grasp()
{
    /* Update the state. */
    GraspExample::SetState(State::StartGrasp);

    return "Command accepted by the module";
}


std::string GraspExample::NewPose(const double x, const double y, const double z, const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration)
{
    /* Update the values. */
    mutex_bottle_.lock();

    cmd_.clear();
    cmd_.addString("NewPose");
    cmd_.addDouble(x);
    cmd_.addDouble(y);
    cmd_.addDouble(z);
    cmd_.addDouble(axis_x);
    cmd_.addDouble(axis_y);
    cmd_.addDouble(axis_z);
    cmd_.addDouble(angle);
    cmd_.addDouble(duration);

    mutex_bottle_.unlock();

    /* Update the state. */
    GraspExample::SetState(State::SendMessage);

    return "Command accepted by the module";
}


std::string GraspExample::NewOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration)
{
    /* Update the values. */
    mutex_bottle_.lock();

    cmd_.clear();
    cmd_.addString("NewOrientation");
    cmd_.addDouble(axis_x);
    cmd_.addDouble(axis_y);
    cmd_.addDouble(axis_z);
    cmd_.addDouble(angle);
    cmd_.addDouble(duration);

    mutex_bottle_.unlock();

    /* Update the state. */
    GraspExample::SetState(State::SendMessage);

    return "Command accepted by the module";
}


std::string GraspExample::NewRelativePosition(const double x, const double y, const double z, const double duration)
{
    /* Update the values. */
    mutex_bottle_.lock();

    cmd_.clear();
    cmd_.addString("NewRelativePosition");
    cmd_.addDouble(x);
    cmd_.addDouble(y);
    cmd_.addDouble(z);
    cmd_.addDouble(duration);

    mutex_bottle_.unlock();

    /* Update the state. */
    GraspExample::SetState(State::SendMessage);

    return "Command accepted by the module";
}


std::string GraspExample::NewRelativeOrientation(const double axis_x, const double axis_y, const double axis_z, const double angle, const double duration, const std::string& fixed_axes)
{
    /* Update the values. */
    mutex_bottle_.lock();

    cmd_.clear();
    cmd_.addString("NewRelativeOrientation");
    cmd_.addDouble(axis_x);
    cmd_.addDouble(axis_y);
    cmd_.addDouble(axis_z);
    cmd_.addDouble(angle);
    cmd_.addDouble(duration);
    cmd_.addString(fixed_axes);

    mutex_bottle_.unlock();

    /* Update the state. */
    GraspExample::SetState(State::SendMessage);

    return "Command accepted by the module";
}


std::string GraspExample::GoHome()
{
    /* Update the state. */
    GraspExample::SetState(State::GoHome);

    return "Command accepted by the module";
}


GraspExample::State GraspExample::GetState()
{
    State state;
    mutex_state_.lock();

    state = state_;

    mutex_state_.unlock();

    return state;
}


bool GraspExample::close()
{
    driver_.close();
    port_rpc_.close();
    port_rpc_client_.close();

    return true;
}


void GraspExample::SetState(GraspExample::State state)
{
    mutex_state_.lock();

    state_ = state;

    mutex_state_.unlock();
}
