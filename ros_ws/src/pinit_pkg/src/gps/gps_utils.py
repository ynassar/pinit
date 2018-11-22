#!/usr/bin/env python

import numpy as np


class GpsPoint():

    def __init__(self, long, lat):
        self.long = long
        self.lat = lat

    def __str__(self):
        return "latitude: {lat} longitude {long}".format(
            long=self.long,
            lat=self.lat
        )
    

def grpc_to_ros(point):
    lat = point.latitude
    long = point.longitude

    return GpsPoint(long, lat)


def coordinates_to_rad(gps_point):
    rad_long = np.radians(gps_point.long)
    rad_lat = np.radians(gps_point.lat)
    rad_gps_point = GpsPoint(long=rad_long, lat=rad_lat)

    return rad_gps_point


def rotate(px, py, theta):
    """ rotates a point around the origin by theta (counterclockwise) """

    rotated_x = np.cos(theta) * px - np.sin(theta) * py
    rotated_y = np.sin(theta) * px + np.cos(theta) * py
    return rotated_x, rotated_y


def polar_to_cartesian(distance, angle):
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)

    return x, y


def get_vector(p1, p2):
    """Calculate the vector between two gps coordinates in radians

    Args:
        p1: first gps coordinates in radians
        p2: second gps coordinate in radians

    Returns:
        d: distance between the two points in meters
        theta: the bearing from one point to another
        #TODO figure out where the bearing start from
    """
    # print "point 1: ", p1, "point2: ", p2
    
    long1 = p1.long
    lat1 = p1.lat
    long2 = p2.long
    lat2 = p2.lat

    R = 6378137

    dLat = lat2 - lat1
    dLong = long2 - long1
    a = np.sin(dLat / 2) * np.sin(dLat / 2) + \
            np.cos(lat1) * np.cos(lat2) * \
            np.sin(dLong / 2) * np.sin(dLong / 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = R * c # meters
    
    y = np.sin(dLong) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - \
            np.sin(lat1) * np.cos(lat2) * np.cos(dLong)
    theta = np.arctan2(y, x)
    
    return d, theta


def convert_gps(dist, theta):
    x = (dist * np.cos(theta))
    y = (dist * np.sin(theta))
    return (x, y)


if __name__=='__main__':
    lat1 = 30.0187716667
    long1 = 31.5006533333
    lat2 = 30.0184083333
    long2 = 31.501105
    p1 = GpsPoint(long=long1, lat=lat1)
    p2 = GpsPoint(long=long2, lat=lat2)
    p1 = coordinates_to_rad(p1)
    p2 = coordinates_to_rad(p2)
    v = get_vector(p1, p2)
    print v[0]
    bearing = np.degrees(v[1])
    bearing = (bearing + 360) % 360
    print bearing 

    dx = lat2 - lat1

