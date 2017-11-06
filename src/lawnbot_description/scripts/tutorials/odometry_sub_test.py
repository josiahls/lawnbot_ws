#!/usr/bin/env python

import rosdep2
import rosdistro
import rosinstall
import rospkg

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def callback(data):
    rospy.loginfo("working...")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def odom_call_back(msg):
    rospy.loginfo("working...")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.pose)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_listener', anonymous=True)
    rospy.loginfo("waiting...")
    rate = rospy.Rate(1)
    #rospy.Subscriber("/gazebo", LaserScan, callback, queue_size=50)
    rospy.Subscriber("/odometry/filtered", Odometry, odom_call_back, queue_size=50)
    rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
