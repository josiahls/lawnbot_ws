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

        direction = 1
        last_x = -1
        last_y = -1

        for node in nodes:
            current_y = state.y
            current_x = state.x
            current_z = state.z

            rospy.loginfo("Traversing new node")

            dest_y = node.state[0]
            dest_x = node.state[1]

            if (last_x or last_y == -1):
                last_x = dest_x
                last_y = dest_y
            else:
                if dest_y - last_y >= 0:
                    direction = 1
                else:
                    direction = -1


            rise_over_run = (current_y - dest_y) / (current_x - dest_x)
            dest_angle = np.arctan(rise_over_run)

            while (abs(current_x - dest_x) > 2) or \
                    (abs(current_y - dest_y) > 2) or \
                    (abs(dest_angle - current_z) > .2):

                while State.lock:
                    print("Locked")

                current_y = state.y
                current_x = state.x
                current_z = state.z

                dest_y = state.y_offset + node.state[0]
                dest_x = state.x_offset + node.state[1]

                dest_angle = np.arctan2(current_y - dest_y, current_x - dest_x)
                dest_angle = dest_angle * direction

                rospy.loginfo("Traversing angle by %s to z %s", dest_angle, current_z)
                rospy.loginfo("Traversing betweenpoints x1 %s y1 %s and x2 %s y2 %s",
                              current_x, current_y, dest_x, dest_y)

                if (abs(dest_angle - current_z) < .2):
                    #cmd_move.linear.x = np.linalg.norm((np.array((current_x, current_y, 0)), np.array((dest_x, dest_y, 0))))
                    sleep(1)
                    cmd_move.linear.x = .5
                    rospy.loginfo("Traversing x by %s", cmd_move.linear.x)
                    move_publisher.publish(cmd_move)
                    sleep(.1)
                else:
                    cmd_turn.angular.z = dest_angle - current_z
                    move_publisher.publish(cmd_turn)
                    sleep(.1)
                sleep(.2)
            last_x = dest_x
            last_y = dest_y
        sleep(3)