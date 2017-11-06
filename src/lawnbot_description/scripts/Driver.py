#!/usr/bin/env python

# This driver is heavily inspired by:
# https://answers.ros.org/question/232216/multiple-subscribers-and-single-publisher-in-one-python-script/
# Sood and tercelkisor deserve credit.
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

from core.State import State

class Driver:

    def __init__(self):
        self.odom_call_ready = False
        self.laser_call_ready = False
        self.image_call_ready = False
        self.goal_core_ready = False
        self.action_core_ready = False
        self.controller_pub_ready = False

    def callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", len(msg.ranges))

    def run(self):

        state = State()
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('jackal_lawnbot_explorer', anonymous=True)
        rospy.loginfo("Waiting to start...")

        # set processing rate
        rospy.Rate(1)

        # init subscribers
        rospy.Subscriber("/odometry/filtered", Odometry, state.odom_callback)
        #rospy.Subscriber("/front/scan", LaserScan, self.callback)
        #rospy.Subscriber("/front/left/image_raw/compressed", Odometry, self.callback)

        while not rospy.is_shutdown():
            try:
                print("Refreshing...")
                plt.clf()
                x, y = np.argwhere(state.state != 0).T
                plt.scatter(x,y)
                plt.pause(0.5)
            except NameError:
                print("well, it WASN'T defined after all!")

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    global state
    plt.ion()
    plt.show()
    dr = Driver()
    dr.run()
