#!/usr/bin/env python

# This driver is heavily inspired by:
# https://answers.ros.org/question/232216/multiple-subscribers-and-single-publisher-in-one-python-script/
# Sood and tercelkisor deserve credit.





import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


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
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('jackal_lawnbot_explorer', anonymous=True)
        rospy.loginfo("Waiting to start...")

        # init subscribers
        rospy.Subscriber("/odometry/filtered", Odometry, self.callback)
        rospy.Subscriber("/front/scan", LaserScan, self.callback)
        rospy.Subscriber("/front/left/image_raw/compressed", Odometry, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    dr = Driver()
    dr.run()
