
def rad(x):
        return x * math.pi / 180

def get_vector(long1, lat1, long2, lat2):
        R = 6378137
        dLat = rad(lat2 - lat1)
        dLong = rad(long2 - long1)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(rad(lat1) * math.cos(rad(lat2) * math.sin(dLong / 2) * math.sin(dLong / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c

        y = math.sin(long2-long1) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat2) * math.cos(lat2) * math.cos(long2-long1)
        theta = math.atan2(y, x)
        
        theta = rad(theta)
        
        return d, theta