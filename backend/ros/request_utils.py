from proto.ros import ros_pb2

import jwt

class AuthenticationError(Exception):
    pass

def UsernameFromToken(token, rsa_key):
    jwt_dict = jwt.decode(token, key=rsa_key)
    return jwt_dict["username"]


def RobotNameFromToken(token, rsa_key):
    jwt_dict = jwt.decode(token, key=rsa_key)
    if not jwt_dict["is_owner"]:
        username = jwt_dict["username"]
        raise AuthenticationError(f"User {username} is not an owner!")
    assert "owned_robot" in jwt_dict
    robot_name = jwt_dict["owned_robot"]
    return robot_name


def RobotNameFromRequest(request, rsa_key):
    if request.HasField("robot_name"):
        return request.robot_name
    elif request.HasField("token"):
        return RobotNameFromToken(request.token, rsa_key)
    else:
        raise AuthenticationError("No robot and no token specified.")


def ConvertWaypointDocumentToProto(document):
    return ros_pb2.Waypoint(waypoint_name=document.waypoint_name,
                            description=document.description)