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
        last_x = state.x
        last_y = state.y

        index = 0
        for node in nodes:
            if index > 1:
                break

            index += 1
            current_y = state.y
            current_x = state.x
            current_z = -1 * state.z

            # If the state is expanding, then that means that
            # the target is out of date
            if State.lock:
                rospy.loginfo("Traverser:call_move state is locked, nodes are invalid")
                break

            dest_y = node.state[0]
            dest_x = node.state[1]

            rospy.loginfo("Traversing from y %s x %s", current_y, current_x)
            rospy.loginfo("Traversing new node %s y %s x %s", node.state, dest_y, dest_x)

            if (last_x or last_y == -1):
                last_x = dest_x
                last_y = dest_y

            dest_angle = np.arctan2(dest_y - current_y, dest_x - current_x)

            while abs(dest_angle + current_z) > .1 and (dest_y != current_y or
                                                            dest_x != current_x):

                current_y = state.y
                current_x = state.x
                current_z = -1 * state.z

                dest_y = node.state[0]
                dest_x = node.state[1]

                dest_angle = np.arctan2(dest_y - current_y, dest_x - current_x)

                rospy.loginfo("Traversing angle by %s to z %s", np.rad2deg(dest_angle), np.rad2deg(current_z))
                #rospy.loginfo("Traversing betweenpoints x1 %s y1 %s and x2 %s y2 %s",current_x, current_y, dest_x, dest_y)

                if (abs(dest_angle + current_z) < .1):
                    #cmd_move.linear.x = np.linalg.norm((np.array((current_x, current_y, 0)), np.array((dest_x, dest_y, 0))))
                    sleep(1)
                    cmd_move.linear.x = .5
                    #rospy.loginfo("Traversing x by %s", cmd_move.linear.x)
                    #move_publisher.publish(cmd_move)
                    #sleep(.1)
                else:
                    cmd_turn.angular.z = dest_angle + current_z
                    move_publisher.publish(cmd_turn)
                    sleep(.1)
                sleep(.2)
            last_x = dest_x
            last_y = dest_y
        sleep(3)