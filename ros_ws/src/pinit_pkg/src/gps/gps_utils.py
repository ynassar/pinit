#!/usr/bin/env python

import numpy as np


class GPS_Point():

    def __init__(self, long, lat):
        self.long = long
        self.lat = lat



def get_vector(p1, p2):
    """Calculate the vector between two gps coordinates

    Args:
        p1: first gps coordinates
        p2: second gps coordinate
        long1: first coordinate longitude in radians
        lat1: first coordinate latitude in radians
        long2: second coordinate longitude in radians
        lat2: second coordinate latitude in radians

    Returns:
        d: distance between the two points in meters
        theta: the bearing from one point to another
        #TODO figure out where the bearing start from
    """

    long1 = p1.long
    lat1 = p1.lat
    long2 = p2.long
    lat2 = p2.lat

    R = 6378137
    # R = 6371000
    dLat = lat2 - lat1
    dLong = long2 - long1
    a = np.sin(dLat / 2) * np.sin(dLat / 2) + \
            np.cos(lat1) * np.cos(lat2) * \
            np.sin(dLong / 2) * np.sin(dLong / 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = R * c

    y = np.sin(dLong) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - \
            np.sin(lat1) * np.cos(lat2) * np.cos(dLong)
    theta = np.arctan2(y, x)

    return d, theta


if __name__=='__main__':
    lat1 = 0
    long1 = 5
    lat2 = 0
    long2 = 3

    lat1 = np.radians(lat1)
    lat2 = np.radians(lat2)
    long1 = np.radians(long1)
    long2 = np.radians(long2)

    v = get_vector(GPS_Point(long=long1, lat=lat1), GPS_Point(long=long2, lat=lat2))
    print v[0] / 1000
    bearing = np.degrees(v[1])
    bearing = (bearing + 360) % 360
    print bearing 
    
