#!/usr/bin/env python

import matplotlib.pyplot as plt
# This driver is heavily inspired by:
# https://answers.ros.org/question/232216/multiple-subscribers-and-single-publisher-in-one-python-script/
# Sood and tercelkisor deserve credit.
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from core.LawnBotProblem import LawnBotProblem
from core.State import State
from core.UninformedSearch import UninformedSearch


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
        rospy.Rate(20)

        # init subscribers
        rospy.Subscriber("/odometry/filtered", Odometry, state.odom_callback)
        rospy.Subscriber("/front/scan", LaserScan, state.laser_callback)
        #rospy.Subscriber("/front/left/image_raw/compressedDepth", LaserScan, state.laser_callback)
        #rospy.Subscriber("/front/left/image_raw/compressed", Odometry, self.callback)

        while not rospy.is_shutdown():

            problem  = LawnBotProblem(initial=np.array((state.x, state.y),int), goal=0, state_space=state.state)

            searcher = UninformedSearch()
            searcher.graph_search(problem)

            try:
                #print("Refreshing...")
                plt.clf()
                x, y = np.argwhere(state.state == 1).T
                plt.scatter(x,y, c="green")
                x, y = np.argwhere(state.state == 2).T
                plt.scatter(x,y, c="blue")
                x, y = np.argwhere(state.state == 3).T
                plt.scatter(x,y, c="red")
                plt.pause(0.5)
            except NameError:
                print("well, it WASN'T defined after all!")
        '''
        while not rospy.is_shutdown():
            try:
                print("Refreshing...")
                x = [None] * len(state.ranges)
                for i in range(len(state.ranges)):
                    x[i] = i

                plt.clf()
                plt.plot(x, state.ranges)
                plt.pause(0.5)
            except NameError:
                print("well, it WASN'T defined after all!")
            except ValueError:
                print("Ranges is not to be shown, x is still not valid!")
            '''
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    global state
    plt.ion()
    plt.show()
    dr = Driver()
    dr.run()
