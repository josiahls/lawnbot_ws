#!/usr/bin/env python

import numpy as np

import rospy
from nav_msgs.msg import Odometry


class State:

    def __init__(self):
        self.odom_call_ready = False
        self.laser_call_ready = False
        self.state = np.zeros((1,1), float)
        self.x_offset = 0
        self.y_offset = 0
        self.is_new = True

    def odom_callback(self, odom=Odometry):
        self.odom_msg = odom
        #rospy.loginfo("State:odom_callback: %s : current position: %s",
         #              rospy.get_caller_id(), self.odom_msg.pose.pose)
        # Get x and y positions from position object
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        # Clean the coordinates
        # Increase their precision, convert to int, add offsets
        x = int(x * 10) + self.x_offset
        y = int(y * 10) + self.y_offset

        rospy.Rate(1)
        '''
        rospy.loginfo("State:odom_callback: %s : current position:  x: %s y :%s \n"
                      " State is: %s rows: %s columns: %s",
                       rospy.get_caller_id(), x, y, self.state.shape,
                      self.state.shape[0],
                      self.state.shape[1])
        
        rospy.loginfo("State:odom_callback: shape: %s", self.state.shape)
        '''

        if y < 0:
            self.state = np.concatenate((np.zeros((abs(y), self.state.shape[1]), float), self.state), axis=0)
            self.y_offset += abs(y)
            y += abs(y)
            rospy.loginfo("State:odom_callback: Adjusting y at %s", self.y_offset)
        if x < 0:
            self.state = np.concatenate((np.zeros((self.state.shape[0], abs(x)), float), self.state), axis=1)
            self.x_offset += abs(x)
            x += abs(x)
            rospy.loginfo("State:odom_callback: Adjusting x at %s", self.x_offset)
        while self.state.shape[0] <= y:
            rospy.loginfo("State:odom_callback: Expanding in the y direction by %s", y - self.state.shape[0])
            self.state = np.concatenate((self.state, np.zeros((1, self.state.shape[1]), float)),axis=0)
        while self.state.shape[1] <= x:
            rospy.loginfo("State:odom_callback: Expanding in the x direction by %s", x - self.state.shape[1])
            self.state = np.concatenate((self.state, np.zeros((self.state.shape[0], 1), float)), axis=1)

        if x >= 0 and y >= 0:
            self.state[y][x] = 1

    def laser_callback(self):
        pass

