#!/usr/bin/env python

import threading
import rospy


class SimpleThread():

    def __init__(self):
        self.thread = None
        self.stop_flag = False
        self.stop_flag_lock = threading.Lock()
        self.loop_function = None
        self.rate = rospy.Rate(10)
        self.thread = threading.Thread(target=self.loop)


    def set_loop_function(self, function):
        self.loop_function = function


    def set_rate(self, rate):
        self.rate = rospy.Rate(rate)


    def set_stop_flag(self, flag):
        self.stop_flag_lock.acquire()
        self.stop_flag = flag
        self.stop_flag_lock.release()


    def get_stop_flag(self):
        self.stop_flag_lock.acquire()
        flag = self.stop_flag
        self.stop_flag_lock.release()

        return flag


    def start(self):
        if not self.thread.is_alive():
            self.thread = threading.Thread(target=self.loop)
            self.thread.start()
        else:
            rospy.loginfo("Thread already running")


    def loop(self):
        while not self.get_stop_flag():
            self.loop_function()
            self.rate.sleep()


    def stop(self):
        self.set_stop_flag(True)
        self.thread.join()
        self.set_stop_flag(False)


if __name__ == "__main__":
    rospy.init_node("ServerStreamerTest")
    streamer = SimpleThread()
    def Test1():
        print "bla"
    streamer.set_loop_function(Test1)
    streamer.start()
    streamer.start()
    rospy.sleep(2)
    streamer.stop()
    rospy.sleep(2)
    streamer.start()
    rospy.sleep(2)
    streamer.stop()
    streamer.start()
    streamer.stop()
