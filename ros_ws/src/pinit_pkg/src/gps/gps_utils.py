#!/usr/bin/env python

import numpy as np



def get_vector(long1, lat1, long2, lat2):
    """Calculate the vector between two gps coordinates
    
    Args:
        long1: first coordinate longitude in radians
        lat1: first coordinate latitude in radians
        long2: second coordinate longitude in radians
        lat2: second coordinate latitude in radians

    Returns:
        d: distance between the two points in meters
        theta: the bearing from one point to another
        #TODO figure out where the bearing start from
    """

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

    v = get_vector(long1, lat1, long2, lat2)
    print v[0] / 1000
    bearing = np.degrees(v[1])
    bearing = (bearing + 360) % 360
    print bearing 
    
