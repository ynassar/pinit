from backend.ros import map_utils
from backend.models import robot as robot_model

class HandlerRegistry(object):
    HANDLERS = {}

    @classmethod
    def RegisterHandler(cls, request_type):
        def RegisterHandlerInner(function):
            cls.HANDLERS[request_type] = function
            return function
        return RegisterHandlerInner

class RosDataHandler(object):
    def __init__(self, map_renderer, ignore_unhandled_types=False):
        self._map_renderer = map_renderer
        self._ignore_unhandled_types = ignore_unhandled_types

    @HandlerRegistry.RegisterHandler("raw_map")
    def HandleRawMapRequest(self, raw_map, robot_name):
        map_array = map_utils.RawMapToArray(raw_map)
        image_array = self._map_renderer.RenderToArray(map_array)
        b64_map_image = map_utils.Base64JpgEncode(image_array)
        map_utils.UpdateMap(robot_name, raw_map, b64_map_image)

    @HandlerRegistry.RegisterHandler("robot_pose")
    def HandleLocalPose(self, robot_pose, robot_name):
        if robot_pose.HasField("timestamp"):
            robot_model.Robot.objects(robot_name=robot_name).update_one(
                upsert=True,
                set__row = robot_pose.row,
                set__column = robot_pose.column,
                set__angle = robot_pose.angle,
                set__timestamp = robot_pose.timestamp
            )
        else:
            robot_model.Robot.objects(robot_name=robot_name).update_one(
                upsert=True,
                set__row = robot_pose.row,
                set__column = robot_pose.column,
                set__angle = robot_pose.angle)

    def HandleRequests(self, request_iterator, robot_name):
        for request in request_iterator:
            request_type = request.WhichOneof("communication")
            if request_type in HandlerRegistry.HANDLERS:
                HandlerRegistry.HANDLERS[request_type](self, getattr(request, request_type), robot_name)
            elif not self._ignore_unhandled_types:
                raise NotImplementedError(f"A handler for {request_type} is not implemented.")