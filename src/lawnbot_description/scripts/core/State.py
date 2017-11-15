#!/usr/bin/env python

from __future__ import print_function
import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion


class State:

    lock = False

    def __init__(self, precision=10):
        self.odom_call_ready = False
        self.laser_call_ready = False
        self.state = np.zeros((1,1), float)
        self.x_offset = 0
        self.y_offset = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.state[self.y][self.x] = 1
        self.precision = precision
        self.max_range = 30
        self.is_new = True
        self.is_ready = True
        self.ranges = []

    def odom_callback(self, odom=Odometry):
        """
        Takes in the odometry object.
        Attempts to set the current position to the state.
        Expands the state if needed.

        :param odom: Odometry object is used in ros for determining the
                     agent's relative position to its starting point
        :return: null
        """
        self.odom_msg = odom
        #rospy.loginfo("State:odom_callback: %s : current position: %s",
         #              rospy.get_caller_id(), self.odom_msg.pose.pose)
        # Get x and y positions from position object
        x = -1 * self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        quaternion = (self.odom_msg.pose.pose.orientation.x,
                      self.odom_msg.pose.pose.orientation.y,
                      self.odom_msg.pose.pose.orientation.z,
                      self.odom_msg.pose.pose.orientation.w)
        self.z = euler_from_quaternion(quaternion)[2]

        #rospy.loginfo("State:odom_callback: z angle: %s", self.z)

        # Clean the coordinates
        # Increase their precision, convert to int, add offsets
        x = int(x * self.precision) + self.x_offset
        y = int(y * self.precision) + self.y_offset

        # Set final current loc
        self.x = x
        self.y = y


        #rospy.loginfo("State:odom_callback : current position:  x: %s y :%s z: %s", x, y, self.z)

        #rospy.loginfo("State:odom_callback: shape: %s", self.state.shape)

        # Checks if the matrix has expanded
        # and / or if it is safe to expand
        if(self.expand_state(y-1,x-1) and self.expand_state(y+1,x+1)):
            for i in range (10):
                # Set as explored
                state_value = self.state[int(y + i * np.sin(self.z))][int(x + -i * np.cos(self.z))]
                if (state_value == 2):
                    break
                if (state_value == 0 or state_value == 3):
                    self.state[int(y + i * np.sin(self.z))][int(x + -i * np.cos(self.z))] = 1

            # add 1 to the current valid location
            if x >= 0 and y >= 0:
                self.state[y][x] = 1

    def laser_callback(self, scan=LaserScan()):
        self.ranges = scan.ranges
        odom_x = self.x
        odom_y = self.y

        #inc = 0
        #for range in self.ranges:
        #plot_x = odom_x + self.ranges[int(len(self.ranges)/2) * np.cos(0)[0]]
        #plot_y = odom_y + self.ranges[int(len(self.ranges)/2) * np.sin(0)[0]]

        '''
                for i in range(len(self.ranges)):
            plot_x = odom_x + self.ranges[int(len(self.ranges)/2)] * \
                              np.cos(i * scan.angle_increment) * self.precision
            
            plot_y = odom_y + self.ranges[int(len(self.ranges)/2)] * \
                              np.sin(i * scan.angle_increment) * self.precision

            if (self.expand_state(plot_y, plot_x)):
                self.state[plot_y][plot_x] = 2
        '''

        #for i in range(len(self.ranges)/2-1, len(self.ranges)/2):
        for i in range(len(self.ranges)/2-14, len(self.ranges)/2+14):
            angle = i - len(self.ranges)/2-1
            # might use cos(i * angle_inc + odom_angleheading
            #rospy.loginfo("State:laser_callback: range: %s at slot %s", self.ranges[i],i)
            #rospy.loginfo("State:laser_callback: adjusted x range: %s", self.ranges[i] * np.cos(0 * scan.angle_increment + self.z) * self.precision)
            #rospy.loginfo("State:laser_callback: adjusted y range: %s", self.ranges[i] * np.sin(0 * scan.angle_increment + self.z) * self.precision)
            #rospy.loginfo("State:laser_callback: adjusted z orientation: %s",self.z)
            #rospy.loginfo("State:laser_callback: scan angle increment: %s", scan.angle_increment)
            #rospy.loginfo("State:laser_callback: max angle %s, min: %s", scan.angle_max, scan.angle_min)
            adjust_x = self.ranges[i] * np.cos(angle * scan.angle_increment + self.z) * self.precision
            adjust_y = self.ranges[i] * np.sin(angle * scan.angle_increment + self.z) * self.precision

            #rospy.loginfo("State:laser_callback: x:%s y:%s",adjust_x,adjust_y)
            # Set adjusted x and y
            plot_x = odom_x + -1 * adjust_x

            plot_y = odom_y + adjust_y

            while State.lock:
                print("Laser is locked out")


            if (self.max_range > np.abs(adjust_x) and
                self.max_range > np.abs(adjust_y)):
                if (self.expand_state(plot_y, plot_x)):
                    self.state[plot_y][plot_x] = 2

        #rospy.loginfo("State:laser_callback: ranges:%s", self.ranges)
        #rospy.loginfo("State:laser_callback: angle_min:%s angle_max:%s angle_inc:%s", scan.angle_min,scan.angle_max, scan.angle_increment)



    def expand_state(self, y, x):
        """
        Safely expands the state matrix.
        Only one process can expand at a time, however
        It will return true for processes that do not require
        expanding.

        :param y: y coordinate of the point that is attempting to be applied
        :param x: x coordinate of the point that is attempting to be applied
        :return: True if the x and y are valid, or has expanded to accommodate them.
                 False if another process is expanding the state already
        """
        lock = True
        y = int(y)
        x = int(x)

        #rospy.loginfo("State:expand_state: self.state.shape[0] > y and self.state.shape[1] > x:%s, %s",
         #             self.state.shape[0] > y, self.state.shape[1] > x)
        if (y >= 0 and x >= 0 and self.state.shape[0] > y and self.state.shape[1] > x):
            return True

        # If the state is not currently being expanded
        if (self.is_ready):
            self.is_ready = False
            # If x or y are negative
            # expand the state left or down
            rospy.loginfo("State:expand_state: y:%s x:%s", y, x)
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
            # If they are larger, expand the state up or right
            while self.state.shape[0] <= y:
                rospy.loginfo("State:odom_callback: Expanding in the y:%s shape: %s direction by %s",y,self.state.shape,  y - self.state.shape[0])
                self.state = np.concatenate((self.state, np.zeros((1, self.state.shape[1]), float)),axis=0)
            while self.state.shape[1] <= x:
                rospy.loginfo("State:odom_callback: Expanding in the x direction by %s", x - self.state.shape[1])
                self.state = np.concatenate((self.state, np.zeros((self.state.shape[0], 1), float)), axis=1)
            # Unlock the method
            self.is_ready = True
            lock = False
            return True
        else:
            lock = False
            return False