"""Implements a service for ROS-related communications."""
import base64
import jwt
import threading
import queue

import cv2
import grpc
import numpy as np
import mongoengine

from backend import constants
from backend.models import map as map_model
from backend.models import waypoint as waypoint_model
from backend.models import robot as robot_model
from backend.models import trip as trip_model
from backend.ros import map_utils
from backend.ros import request_utils
from backend.ros import ros_data_handler
from backend.ros import greyscale_map_renderer
from proto.ros import ros_pb2_grpc
from proto.ros import ros_pb2

TESTING_ROBOT = True

class NotAwaitingConfirmation(Exception):
    pass

class RobotBusy(Exception):
    pass

class NoMapFound(Exception):
    pass

class RobotNotConnected(Exception):
    pass

class NoKnownRobotPosition(Exception):
    pass

class WaypointAlreadyExists(Exception):
    pass

class AuthenticationError(Exception):
    pass


def BuildRosService(ignore_unhandled_communication_types=False):
    """Factory method for RosService instances."""
    map_renderer = greyscale_map_renderer.GreyscaleMapRenderer()
    handler = ros_data_handler.RosDataHandler(map_renderer, ignore_unhandled_communication_types)
    return RosService(handler, constants.DEFAULT_RSA_KEY)

class RosService(ros_pb2_grpc.RosServiceServicer):
    """A servicer for the RosService, which implements RPCs related to ROS communication."""
    def __init__(self, ros_data_handler, rsa_key):
        self._robot_name_to_queue = {}
        self._ros_data_handler = ros_data_handler
        self._rsa_key = rsa_key

    def Communicate(self, request_iterator, context):
        """Handles two-way communication to and from a robot.
        
        Incoming data is handled by self._ros_data_handler, and outgoing messages should be
        pushed to self._robot_name_to_queue[robot_name], where robot_name is the name of the
        robot to send the message to.

        The first message sent by the robot should contain the robot name for identification.
        """
        first_request = next(request_iterator)
        robot_name = first_request.robot_name
        threading.Thread(
            target=self._ros_data_handler.HandleRequests,
            args=(request_iterator, robot_name)
        ).start()

        if robot_name not in self._robot_name_to_queue:
            self._robot_name_to_queue[robot_name] = queue.Queue()
        communication_queue = self._robot_name_to_queue[robot_name]
        while context.is_active():
            try:
                communication = communication_queue.get_nowait()
                yield communication
            except queue.Empty:
                pass
        del self._robot_name_to_queue[robot_name]

    def SendMovement(self, request, context):
        robot_name = request_utils.RobotNameFromRequest(request, self._rsa_key)
        if robot_name not in self._robot_name_to_queue:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Specified robot not connected.")
            raise RobotNotConnected()
        else:
            communication_queue = self._robot_name_to_queue[robot_name]
            if request.mapping_request.request_type == ros_pb2.ServerToRosMappingRequest.START_MAPPING:
                map_model.Map.objects(robot_name = robot_name).delete()
                waypoint_model.Waypoint.objects(robot_name=robot_name).delete()
            communication_queue.put(ros_pb2.ServerToRosCommunication(
                mapping_request=request.mapping_request
            ))
            return ros_pb2.MappingResponse()

    def GetMapImage(self, request, context):
        robot_name = request_utils.RobotNameFromRequest(request, self._rsa_key)
        try:
            map = map_model.Map.objects.get(robot_name = robot_name)
            return ros_pb2.MapImage(resolution=map.resolution, encoded_image=map.b64_image)
        except mongoengine.DoesNotExist:
            raise NoMapFound()
    
    def GetRawMap(self, request, context):
        robot_name = request_utils.RobotNameFromRequest(request, self._rsa_key)
        try:
            map = map_model.Map.objects.get(robot_name = request.robot_name)
            return map_utils.RawMapFromMapDocument(map)
        except mongoengine.DoesNotExist:
            raise NoMapFound()

    def AddWaypoint(self, request, context):
        robot_name = request_utils.RobotNameFromToken(request.token, self._rsa_key)
        try:
            robot = robot_model.Robot.objects.get(robot_name = robot_name)
        except mongoengine.DoesNotExist:
            raise NoKnownRobotPosition()
        try:
            waypoint_model.Waypoint(waypoint_name = request.waypoint_name,
                                    description = request.description,
                                    robot_name = robot_name,
                                    row=robot.row,
                                    column=robot.column).save()
        except mongoengine.NotUniqueError:
            raise WaypointAlreadyExists()
        return ros_pb2.AddWaypointResponse()

    def GetNearbyWaypoints(self, request, context):
        location = [request.longitude, request.latitude]

        nearby_maps = map_model.Map.objects(origin__near=location)
        if not nearby_maps:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("No map found near given location.")
            raise NoMapFound()
        nearby_map = nearby_maps[0]
        robot_name = nearby_map.robot_name
        nearby_waypoints = waypoint_model.Waypoint.objects(robot_name=robot_name)
        waypoint_protos = [request_utils.ConvertWaypointDocumentToProto(document)
                           for document in nearby_waypoints]
        if TESTING_ROBOT:
            self._robot_name_to_queue[robot_name].put(ros_pb2.ServerToRosCommunication(navigation_request=ros_pb2.ServerToRosNavigationRequest(
                pose=ros_pb2.LocalMapPose(row=nearby_waypoints[0].row, column=nearby_waypoints[0].column)
            )))
        return ros_pb2.WaypointList(waypoints=waypoint_protos)

    def GetPose(self, request, context):
        robot = robot_model.Robot.objects().get(robot_name=request.robot_name)
        return ros_pb2.LocalMapPose(row=robot.row, column=robot.column, angle=robot.angle)

    def CreateTrip(self, request, context):
        username = request_utils.UsernameFromToken(request.token, self._rsa_key)
        start_waypoint = waypoint_model.Waypoint.objects.get(waypoint_name=request.start_waypoint)
        robot_name = start_waypoint.robot_name
        robot = robot_model.Robot.objects.get(robot_name=robot_name)
        if robot.status != 'Idle':
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details('Robot is busy.')
            raise RobotBusy()
        if robot_name not in self._robot_name_to_queue:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Specified robot not connected.")
            raise RobotNotConnected()
        trip = trip_model.Trip(created_by=username,
                               start_waypoint=request.start_waypoint,
                               end_waypoint=request.end_waypoint,
                               status='RoutingToPickup')
        robot.trip = trip
        trip.save()
        robot.save()
        self._robot_name_to_queue[robot_name].put(ros_pb2.ServerToRosCommunication(
            navigation_request=ros_pb2.ServerToRosNavigationRequest(
                pose=ros_pb2.LocalMapPose(
                    row=start_waypoint.row,
                    column=start_waypoint.column
                )
            )
        ))
        return ros_pb2.CreateTripResponse()

    def GetTripStatus(self, request, context):
        username = request_utils.UsernameFromToken(request.token, self._rsa_key)
        trip = trip_model.Trip.objects(created_by=username).order_by('-id').first()+
        status_string_to_enum = {
            'RoutingToPickup' : ros_pb2.TripStatus.ROUTING_TO_PICKUP,
            'AwaitingConfirmation' : ros_pb2.TripStatus.AWAITING_CONFIRMATION,
            'RoutingToDestination' : ros_pb2.TripStatus.ROUTING_TO_DESTINATION,
            'Completed' : ros_pb2.TripStatus.COMPLETED
        }
        return ros_pb2.TripStatus(status=status_string_to_enum[trip.status])

    def ConfirmTrip(self, request, context):
        username = request_utils.UsernameFromToken(request.token, self._rsa_key)
        trip = trip_model.Trip.objects.get(created_by=username, status__ne='Completed')
        if trip.status != 'AwaitingConfirmation':
            context.set_code(grpc.StatusCode.NOT_FOUND)
            raise NotAwaitingConfirmation()
        end_waypoint = waypoint_model.Waypoint.objects.get(waypoint_name=trip.end_waypoint)
        trip.status = 'RoutingToDestination'
        robot_name = end_waypoint.robot_name
        if robot_name not in self._robot_name_to_queue:
            context.set_code(grpc.StatusCode.NOT_FOUND)
            context.set_details("Specified robot not connected.")
            raise RobotNotConnected()
        self._robot_name_to_queue[robot_name].put(ros_pb2.ServerToRosCommunication(
            navigation_request=ros_pb2.ServerToRosNavigationRequest(
                pose=ros_pb2.LocalMapPose(
                    row=end_waypoint.row,
                    column=end_waypoint.column
                )
            )
        ))
        trip.save()
        return ros_pb2.ConfirmTripResponse()