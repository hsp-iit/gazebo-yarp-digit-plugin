/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

 #include "GraspExample.h"

 #include <yarp/os/ResourceFinder.h>
 #include <yarp/os/LogStream.h>


int main (int argc, char* argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork())
    {
        yError() << "YARP network not available";

        return EXIT_FAILURE;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    GraspExample module;

    return module.runModule(rf);
}
