#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from time import sleep

import rospy
from geometry_msgs.msg import Twist

from State import State


class Traverser(object):
    def call_starting_move(self, state, move_publisher=rospy.Publisher):
        cmd_turn = Twist()
        current_z = state.z
        cmd_turn.angular.z = 4
        move_publisher.publish(cmd_turn)
        sleep(1.5)

    def call_move(self, nodes, state, move_publisher=rospy.Publisher):

        cmd_move = Twist()
        cmd_turn = Twist()
        """
        rospy.loginfo("Scanning Environment ")
        for i in range(1, 360, 20):
            cmd_turn.angular.z = i * 0.0023
            move_publisher.publish(cmd_turn)
            sleep(.3)
        """
        self.last_x = state.x
        self.last_y = state.y
        self.dest_x = state.x
        self.dest_y = state.y

        index = 0
        for node in nodes:
            if index > 2:
                break

            index += 1

            self.update(node, state)
            # If the state is expanding, then that means that
            # the target is out of date
            if State.lock:
                rospy.loginfo("Traverser:call_move state is locked, nodes are invalid")
                break

            rospy.loginfo("Traversing from y %s x %s", self.last_y, self.last_x)
            rospy.loginfo("Traversing new node %s y %s x %s", node.state, self.dest_y, self.dest_x)
            rospy.loginfo("Traversing angle by %s to z %s", np.rad2deg(self.dest_angle), np.rad2deg(self.current_z))
            while abs(self.dest_angle - self.current_z) > .1:
                rospy.loginfo("Traversing angle by %s to z %s", np.rad2deg(self.dest_angle), np.rad2deg(self.current_z))
                self.update(node, state)


                #rospy.loginfo("Traversing betweenpoints x1 %s y1 %s and x2 %s y2 %s",current_x, current_y, dest_x, dest_y)

                if (abs(self.dest_angle - self.current_z) < .1):
                    #cmd_move.linear.x = np.linalg.norm((np.array((current_x, current_y, 0)), np.array((dest_x, dest_y, 0))))
                    sleep(1)
                    cmd_move.linear.x = .1
                    #rospy.loginfo("Traversing x by %s", cmd_move.linear.x)
                    #move_publisher.publish(cmd_move)
                    #sleep(.1)
                else:
                    if (self.dest_angle - self.current_z < 0):
                        cmd_turn.angular.z = .5
                    else:
                        cmd_turn.angular.z = -.5
                    move_publisher.publish(cmd_turn)
                    sleep(.1)
                sleep(.2)
        sleep(3)


    def update(self, node, state):
        self.current_y = state.y
        self.current_x = state.x
        self.current_z = -1 * state.z

        if (node.state[0] != self.last_y and
            node.state[1] != self.last_x and
            node.state[0] != self.dest_y and
            node.state[1] != self.dest_x):
            self.last_y = self.dest_y
            self.last_x = self.dest_x
            self.dest_y = node.state[0]
            self.dest_x = node.state[1]

        self.dest_angle = np.arctan2(self.dest_y - self.last_y,
                                self.dest_x - self.last_x)

        if self.dest_angle > 0:
            self.dest_angle -= np.deg2rad(180)
        else:
            self.dest_angle += np.deg2rad(180)