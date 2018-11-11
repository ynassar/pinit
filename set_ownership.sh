# Sets a given user's owner status and assigns a robot.
# Invoke as "bash set_ownership.sh <username> <robot_name>"

mongo local --eval "db.user.update({\"username\":\"$1\"}, {\$set:{\"is_owner\":true,\"owned_robot\":\"$2\"}})"