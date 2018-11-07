import jwt

class AuthenticationError(Exception):
    pass

def RobotNameFromRequest(request, rsa_key):
    if request.HasField("robot_name"):
        robot_name = request.robot_name
    elif request.HasField("token"):
        jwt_dict = jwt.decode(request.token, key=rsa_key)
        if not jwt_dict["is_owner"]:
            username = jwt_dict["username"]
            raise AuthenticationError(f"User {username} is not an owner!")
        assert "owned_robot" in jwt_dict
        robot_name = jwt_dict["owned_robot"]
    else:
        raise AuthenticationError("No robot and no token specified.")
    return robot_name