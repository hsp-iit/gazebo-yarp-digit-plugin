/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

service SetNewPoseIDL
{
    string NewPose(1: double x, 2: double y, 3: double z, 4: double axis_x, 5: double axis_y, 6: double axis_z, 7: double angle, 8: double duration);

    string NewOrientation(1: double axis_x, 2: double axis_y, 3: double axis_z, 4: double angle, 5: double duration);

    string NewPosition(1: double x, 2: double y, 3: double z, 4: double duration);

    string NewRelativePosition(1: double x, 2: double y, 3: double z, 4: double duration);

    string NewRelativeOrientation(1: double axis_x, 2: double axis_y, 3: double axis_z, 4: double angle, 5: string fixed_axes, 6: double duration);

    string SetStaticPose(1: double x, 2: double y, 3: double z, 4: double axis_x, 5: double axis_y, 6: double axis_z, 7: double angle);

    string SetStaticPosition(1: double x, 2: double y, 3: double z);

    string SetStaticOrientation(1: double axis_x, 2: double axis_y, 3: double axis_z, 4: double angle);

    string GoHome();

    string SetControlStatus(1: string status);
}
