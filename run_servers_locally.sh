# Runs all servers locally.

# Should ONLY be run from the pinit root directory.


source install.sh > /dev/null
source run_codegen.sh > /dev/null

export PYTHONPATH=$PYTHONPATH:$PWD

source .env/backend/bin/activate

printf "\n\n------------------ Starting Servers ------------------\n\n"

python3 backend/account_server.py &
ACCOUNT_SERVER_PID=$!

python3 backend/ros/ros_communication_server.py &
ROS_SERVER_PID=$!

trap "kill $ACCOUNT_SERVER_PID && kill $ROS_SERVER_PID" EXIT

wait

deactivate