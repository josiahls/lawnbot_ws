#!/usr/bin/env python

from __future__ import print_function
from __future__ import print_function
import rosdep2
import rosdistro
import rosinstall
import rospkg

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

def callback(msg):
    rospy.loginfo("working...")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", len(msg.ranges))
    global ranges
    ranges = msg.ranges

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_listener', anonymous=True)
    rospy.loginfo("waiting...")
    rate = rospy.Rate(1)
    rospy.Subscriber("/scan", LaserScan, callback, queue_size=50)

    while not rospy.is_shutdown():
        try:
            print("Refreshing...")
            x = [None] * len(ranges)
            for i in range(len(ranges)):
                x[i] = i

            plt.clf()
            plt.plot(x, ranges)
            plt.pause(0.5)
        except NameError:
            print("well, it WASN'T defined after all!")

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped



if __name__ == '__main__':
    global state
    plt.ion()
    plt.show()
    listener()
