#!/bin/bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]
then
        echo "Please source the script instead of just executing it"
        echo "Run command \"source ${0}\""
        exit
fi

echo "Getting ip addr..."
ip="$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')"
echo "IP = $ip"
echo "Exporting ros ip..."
export ROS_IP=$ip
echo "Done!"

echo "Exporting PYTHONPATH..."
export PYTHONPATH=$PYTHONPATH:~/pinit
export PYTHONPATH=$PYTHONPATH:~/pinit/ros_ws/src/pinit_pkg/src
echo "Done!"

