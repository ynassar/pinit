#!/usr/bin/env python
import robot_motion.robot_pose

import rospy
import nav_msgs.msg as ros_nav_msgs
import geometry_msgs.msg as ros_geo_msgs

class MapRePublisher():

    @classmethod
    def create(cls, pose_listener):
        return MapRePublisher(pose_listener)


    def __init__(self, pose_listener):
        self.map_listener_topic_name = 'map'
        self.map_publish_topic_name = 'map2'
        self.pose_listener = pose_listener
        self.subscriber = rospy.Subscriber(self.map_listener_topic_name,
                                           ros_nav_msgs.OccupancyGrid,
                                           self.map_callback)
        self.publisher = rospy.Publisher(self.map_publish_topic_name,
                                         ros_nav_msgs.OccupancyGrid,
                                         queue_size=4)
        self.pose_publisher = rospy.Publisher('pose',
                                         ros_geo_msgs.PoseStamped,
                                         queue_size=4)


    def map_callback(self, grid):
        pose = self.pose_listener.get_pose()
        self.pose_publisher.publish(pose)
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y

        #new_grid = self.update_grid(pose_x, pose_y, grid)
        new_grid = self.test_grid()
        self.publish_map(new_grid)


    def test_grid(self):
        grid = ros_nav_msgs.OccupancyGrid()
        grid.info.resolution = 1
        grid.info.origin.position.x = -100
        grid.info.origin.position.y = -100
        width = 200
        grid.info.width = width
        grid.info.height = width
        grid.header.frame_id = "map_test"
        empty_data = [-1 for i in range(width * width)]
        empty_data = tuple(empty_data)
        x_min = (-1 * grid.info.origin.position.x + -1) / grid.info.resolution
        x_max = (-1 * grid.info.origin.position.x + 10) / grid.info.resolution
        y_min = (-1 * grid.info.origin.position.y + 0) / grid.info.resolution
        y_max = (-1 * grid.info.origin.position.y + 1) / grid.info.resolution
        grid.data = tuple(self.fill_square(x_min, y_min, x_max, y_max, list(empty_data), width))
        return grid


    def fill_square(self, x_min, y_min, x_max, y_max, flat_grid, width):
        len_x = abs(int(x_max - x_min))
        len_y = abs(int(y_max - y_min))

        for i in range(len_x):
            for j in range(len_y):
                y = y_min + j
                x = x_min + i
                idx = self.get_map_idx(x, y, width)
                if idx < len(flat_grid) - 1:
                    flat_grid[idx] = 100
        return flat_grid


    def get_map_idx(self, width, x, y):
        return int(width * y + x)


    def update_grid(self, x, y, grid):
        width = grid.info.width
        reso = grid.info.resolution
        grid_x = (x - grid.info.origin.position.x) / reso
        grid_y = (y - grid.info.origin.position.y) / reso
        idx = self.get_map_idx(width, grid_x, grid_y)
        grid_data_list = list(grid.data)
        print idx, x, y, grid_x, grid_y
        for i in range(40):
            grid_data_list[self.get_map_idx(width, grid_x + i, grid_y)] = 100
        for i in range(40):
            grid_data_list[self.get_map_idx(width, grid_x - i, grid_y)] = 100
        for i in range(40):
            grid_data_list[self.get_map_idx(width, grid_x, grid_y + i)] = 100
        for i in range(40):
            grid_data_list[self.get_map_idx(width, grid_x, grid_y - i)] = 100
        grid_data_tuple = tuple(grid_data_list)
        grid.data = grid_data_tuple
        return grid


    def publish_map(self, grid):
        grid.header.frame_id = self.map_listener_topic_name
        self.publisher.publish(grid)


if __name__ == "__main__":
    rospy.init_node("map_republisher")
    pose_listener = robot_motion.robot_pose.PoseListenerFactory()
    map_republisher = MapRePublisher.create(pose_listener)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()
