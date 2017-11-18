#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from time import sleep

import rospy
from geometry_msgs.msg import Twist

from State import State


class Traverser(object):
    """
    I found this URL useful for localizing angles:
    <a>http://edspi31415.blogspot.com/2013/11/atan2-using-tan-1-and-anglearg-various.html </a>
    """

    def call_starting_move(self, state, move_publisher=rospy.Publisher):

        self.cmd_turn = Twist()
        current_z = state.z
        self.cmd_turn.angular.z = 4
        move_publisher.publish(self.cmd_turn)
        sleep(1.5)

    def call_move(self, nodes, state, move_publisher=rospy.Publisher):

        self.cmd_move = Twist()
        self.cmd_turn = Twist()
        """
        rospy.loginfo("Scanning Environment ")
        for i in range(1, 360, 20):
            cmd_turn.angular.z = i * 0.0023
            move_publisher.publish(cmd_turn)
            sleep(.3)
        """

        starting_index = self.get_closest_node(nodes, state)

        self.last_x = state.x
        self.last_y = state.y
        self.dest_x = state.x
        self.dest_y = state.y

        index = -1
        for node in nodes:
            index += 1
            if (index >= starting_index):
                self.update(node, state)
                # If the state is expanding, then that means that
                # the target is out of date
                if State.lock:
                    rospy.loginfo("Traverser:call_move state is locked, nodes are invalid")
                    break

                rospy.loginfo("Traversing from y %s x %s", self.last_y, self.last_x)
                rospy.loginfo("Traversing new node %s y %s x %s", node.state, self.dest_y, self.dest_x)

                try:
                    self.dest_angle
                except:
                    continue

                # rospy.loginfo("Traversing angle by %s to z %s", self.dest_angle, self.current_z)
                location_reached = False
                while abs(self.dest_angle - self.current_z) > .1 or not location_reached:
                    # rospy.loginfo("Traversing angle by %s to z %s", self.dest_angle, self.current_z)
                    self.update(node, state)

                    # rospy.loginfo("Traversing betweenpoints x1 %s y1 %s and x2 %s y2 %s",current_x, current_y, dest_x, dest_y)

                    if (abs(self.dest_angle - self.current_z) <= .1 and not location_reached):
                        # cmd_move.linear.x = np.linalg.norm((np.array((current_x, current_y, 0)), np.array((dest_x, dest_y, 0))))

                        # rospy.loginfo("Traversing x by %s", self.cmd_move.linear.x)
                        max = self.max
                        while self.max <= max:
                            max = self.max
                            self.cmd_move.linear.x = .2
                            move_publisher.publish(self.cmd_move)
                            self.update(node, state)
                            # rospy.loginfo("Traversing x by %s where max is %s and self.max is %s", self.cmd_move.linear.x, max, self.max)
                        location_reached = True
                    else:
                        self.cmd_turn.angular.z = self.get_angle()
                        move_publisher.publish(self.cmd_turn)
                else:
                    rospy.loginfo("Traverser:call_move: skipping index %s", index)

    def get_angle(self):
        angle = 0

        if (np.rad2deg(self.dest_angle) - np.rad2deg(self.current_z) > 0):
            angle = .5
        else:
            angle = -.5

        if (abs(self.dest_angle - self.current_z) < .4 and abs(self.cmd_turn.angular.z) > .2):
            angle = self.cmd_turn.angular.z * .999
            # rospy.loginfo("Traverser:get_angle: adjusting angle: %s", angle)

        return angle

    def get_closest_node(self, nodes, state):

        closest_node_distance = 0
        closest_node_index = 0
        node_index = 0

        for node in nodes:
            distance = np.sqrt(np.power(node.state[1] - state.x, 2) + np.power(node.state[0] - state.y, 2))
            if node_index == 0:
                closest_node_distance = distance
                # Node is too far away to safely blindly traverse
                if closest_node_distance > 3:
                    closest_node_index = -1
                    break
            elif (distance < closest_node_distance):
                closest_node_distance = distance
                closest_node_index = node_index
            node_index += 1

        return closest_node_index

    def update(self, node, state):
        self.current_y = state.y
        self.current_x = state.x
        self.current_z = state.z

        # rospy.loginfo("Node y: %s x: %s Last: y: %s x: %s Dest: y: %s x: %s", node.state[0], node.state[1], self.last_y, self.last_x, self.dest_y, self.dest_x)

        self.max = np.sqrt(np.power(self.current_x - self.dest_x, 2) + np.power(self.current_y - self.dest_y, 2))

        # if the last and destination are the same
        # and the current x y is not the last destination
        # try to make the last one the current location
        # so that the agent will try to move on track
        if (self.last_x == self.dest_x and
                    self.last_y == self.dest_y and
                    self.last_x != self.current_x and
                    self.last_y != self.current_y):
            self.last_x = self.current_x
            self.last_y = self.current_y

        # if the destination is not the goal location
        # then make it the goal location
        # and move the previous destination back
        if (self.dest_y != node.state[0] or
                    self.dest_x != node.state[1]):
            self.last_y = self.dest_y
            self.last_x = self.dest_x
            self.dest_y = node.state[0]
            self.dest_x = node.state[1]

        # If the points are different, change the angle
        if (self.last_x != self.dest_x or self.last_y != self.dest_y):
            # rospy.loginfo("Adjusting Angle Node y: %s x: %s Last: y: %s x: %s Dest: y: %s x: %s",node.state[0], node.state[1], self.last_y, self.last_x, self.dest_y, self.dest_x)
            # Note these are flipped because the graph directions are flipped
            quad_y = self.dest_y - self.last_y
            quad_x = self.dest_x - self.last_x

            # quad_y = -1
            # quad_x = 1

            # rospy.loginfo("Traverser:update:quad_y %s quad_x: %s", quad_y, quad_x)

            # Quadrant 2
            if (quad_y > 0 and quad_x < 0):
                self.dest_angle = np.arctan2(quad_y, quad_x) + np.pi / 2
            elif (quad_y < 0 and quad_x < 0):
                self.dest_angle = np.arctan2(quad_y, quad_x) + np.pi / 2
            # Quadrant 4
            elif (quad_y < 0 and quad_x > 0):
                self.dest_angle = np.arctan2(quad_y, quad_x) - np.pi / 2
            else:
                self.dest_angle = np.arctan2(quad_y, quad_x) + np.pi / 2

            # If the generated quadrant in more than a full circle
            if abs(self.dest_angle) > np.pi:
                self.dest_angle = self.dest_angle - np.pi if self.dest_angle > np.pi else self.dest_angle + np.pi

            # If any of the coordinates are 0, handle this
            if quad_x == 0 and quad_y < 0:
                self.dest_angle = - np.pi / 2
            if quad_x == 0 and quad_y > 0:
                self.dest_angle = np.pi / 2
            if quad_y == 0 and quad_x < 0:
                self.dest_angle = 0
            if quad_y == 0 and quad_x > 0:
                self.dest_angle = np.pi
