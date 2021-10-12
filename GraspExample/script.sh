#!/bin/bash
echo "NewRelativePosition 0.35 -0.04 0 3" | yarp rpc /grasp-example/rpc:i
sleep 4s
echo "NewRelativeOrientation 1 0 0 -1.5 \"false\" 3" | yarp rpc /grasp-example/rpc:i
sleep 4s
echo "NewRelativePosition 0.0 0.0 -0.04 3.0" | yarp rpc /grasp-example/rpc:i
sleep 4s
echo "NewRelativePosition 0.003 0 0 2" | yarp rpc /grasp-example/rpc:i
sleep 3s
echo "Grasp" | yarp rpc /grasp-example/rpc:i
