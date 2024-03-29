from proto.ros import ros_pb2
from backend.models import map as map_model

import base64
import cv2
import numpy as np


def RawMapToArray(raw_map):
    map_data = raw_map.data
    map_height = raw_map.height
    map_width = raw_map.width
    return np.frombuffer(map_data, dtype='uint8').reshape(map_height, map_width)


def Base64JpgEncode(image_array):
    _, encoded_image = cv2.imencode('.jpg', image_array)
    b64_image = base64.b64encode(encoded_image).decode('utf-8')
    return b64_image


def UpdateMap(robot_name, raw_map, b64_map_image):
    origin = [raw_map.origin.longitude, raw_map.origin.latitude]
    map_model.Map.objects(robot_name=robot_name).update_one(
        upsert=True,
        set__resolution=raw_map.resolution,
        set__b64_image=b64_map_image,
        set__raw_map=raw_map.data.decode('utf-8'),
        set__height=raw_map.height,
        set__width=raw_map.width,
        set__origin=origin,
        set__origin_angle_shift=raw_map.origin_angle_shift,
        set__shift_x=raw_map.shift_x,
        set__shift_y=raw_map.shift_y)


def RawMapFromMapDocument(map):
    origin = ros_pb2.GpsCoordinates(longitude=map.origin['coordinates'][0],
                                    latitude=map.origin['coordinates'][1])
    return ros_pb2.RawMap(resolution=map.resolution, 
                   height=map.height,
                   width=map.width,
                   data=map.raw_map.encode('utf-8'),
                   origin_angle_shift=map.origin_angle_shift,
                   origin=origin,
                   shift_x=map.shift_x,
                   shift_y=map.shift_y)