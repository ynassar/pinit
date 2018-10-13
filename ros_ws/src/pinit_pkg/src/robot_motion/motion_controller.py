import rospy
from geometry_msgs import Twist

class MotionController():

    def __init__(self):
        self.node_name = ''
        self.pub_name = ''
        self.rate = 0
        self.vel_x = 0
        self.vel_theta = 0
        self.vel_pub = rospy.Publisher(self.pub_name, 
                                        Twist,
                                        queue_size=10)
    

    def init_node(self):
        rospy.init_node(self.node_name, anonymous=True)



if __name__ == '__main__':
    controller = MotionController()
    controller.init_node()