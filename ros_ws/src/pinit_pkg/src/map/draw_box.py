#!/usr/bin/env python
import rospy
import tf
import nav_msgs.msg as ros_nav_msgs
import geometry_msgs.msg as ros_geo_msgs


def get_tf_trans(listener):
    trans = None
    try:
        listener.waitForTransform('/map', '/tf_tests', rospy.Time(0), rospy.Duration(2))
        (trans, rot) = listener.lookupTransform('/map', '/tf_tests', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print e
    return trans, rot


def meter_vector_to_coordinate(map_metadata, meters_x, meters_y):
    origin_x = map_metadata.origin.position.x
    origin_y = map_metadata.origin.position.y
    resolution = map_metadata.resolution

    x_coordinate = int((meters_x - origin_x) / resolution)
    y_coordinate = int((meters_y - origin_y) / resolution)

    return (x_coordinate, y_coordinate)


def draw_box_flat_map(map, length, width,
        box_bottom_right_corner_x, box_bottom_right_corner_y): #meters
    map_metadata = map.info
    map_data_tuple = map.data
    map_data_list = list(map_data_tuple)
    map_width = map_metadata.width
    x_min, y_min = meter_vector_to_coordinate(map_metadata,
            box_bottom_right_corner_x,
            box_bottom_right_corner_y)

    x_max, y_max = meter_vector_to_coordinate(map_metadata,
            box_bottom_right_corner_x + length,
            box_bottom_right_corner_y + width)

    len_x = (x_max - x_min)
    len_y = (y_max - y_min)
    for i in range(len_x):
        for j in range(len_y):
            x = x_min + i
            y = y_min + j
            idx = (map_width * y + x)
            if idx < len(map_data_list):
                map_data_list[idx] = 100
    
    map_data_tuple = tuple(map_data_list)
    map.data = map_data_tuple

    return map


class MapDrawer():
    def __init__(self):
        self.map_listener_topic_name = 'map'
        self.map_publish_topic_name = 'map2'
        self.publisher = rospy.Publisher(self.map_publish_topic_name,
                                         ros_nav_msgs.OccupancyGrid,
                                         queue_size=4)
        self.subscriber = rospy.Subscriber(self.map_listener_topic_name,
                                           ros_nav_msgs.OccupancyGrid,
                                           self.map_callback)
        self.pose_publisher = rospy.Publisher('pose',
                                         ros_geo_msgs.PoseStamped,
                                         queue_size=4)


    def map_callback(self, map):
        tf_listener = tf.TransformListener()
        trans, rot = get_tf_trans(tf_listener)
        while trans is None:
            trans, rot = get_tf_trans(tf_listener)
        pose_msg = ros_geo_msgs.PoseStamped()
        pose_msg.pose.position = ros_geo_msgs.Point(trans[0], trans[1], trans[2])
        pose_msg.pose.orientation = ros_geo_msgs.Quaternion(rot[0], rot[1], rot[2], rot[3])
        pose_msg.header.frame_id = "map2"
        self.pose_publisher.publish(pose_msg)
        x = trans[0]
        y = trans[1]

        length = 1
        width = 1

        map = draw_box_flat_map(map, length, width, x, y)
        map.header.frame_id = "map2"
        self.publisher.publish(map)



    


if __name__ == "__main__":
    rospy.init_node("draw_box")
    drawer = MapDrawer()
    rospy.spin()
    
