#!/usr/bin/env python

import rosdep2
import rosdistro
import rosinstall
import rospkg

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


def callback(data):
    print "working..."
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_listener', anonymous=True)
    print "waiting..."
    rospy.Subscriber("/front/scan/LaserScan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
