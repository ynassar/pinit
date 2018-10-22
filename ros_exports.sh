#!/bin/bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]
then
        echo "Please source the script instead of just executing it"
        echo "Run command \"source ${0}\""
        exit
fi

if [[ "${PWD##*/}" != "pinit" ]]
then
	echo "Please source the script from the project root directory"
	return
fi

echo "Getting ip addr..."
ip="$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')"
echo "IP = $ip"
echo "Exporting ros ip..."
export ROS_IP=$ip
echo "Done!"

echo "Exporting PYTHONPATH..."
export PYTHONPATH=$PYTHONPATH:${PWD}
export PYTHONPATH=$PYTHONPATH:${PWD}/ros_ws/src/pinit_pkg/src
echo "Done!"

