#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import rospy

from State import State
from SearchNode import SearchNode


class LawnBotProblem(object):
    """The abstract class for a formal problem.  You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=0, state_space=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        # rospy.loginfo("Initial Node: %s", str(initial))
        self.initial = initial
        self.goal = goal
        self.state_space = state_space
        self.action_padding = 4
        self.h_location = [0, 0]
        self.gen_h(initial)

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        # print("actions being called")

        # rospy.loginfo("Actions:state: %s", str((state[0], state[1])))
        # rospy.loginfo("Actions:state space: %s", str(self.state_space.shape))
        actions = np.asarray([[0, 0]], int)
        for g in range(1, self.action_padding):
            for i in range(0, 360, 45):
                while State.lock:
                    # print("State is locked")
                    pass

                #adjust_y = int(np.around(np.sin(np.deg2rad(i))))
                #adjust_x = int(np.around(np.cos(np.deg2rad(i))))

                angle = i if i <= 180 else -360 + i
                adjust_y = np.sin(np.deg2rad(angle))
                adjust_x = np.cos(np.deg2rad(angle))

                new_y = state[0] + int(g * adjust_y)
                new_x = state[1] + int(g * adjust_x)

                #rospy.loginfo("Actions: x %s y %s for i %s", str(adjust_x),str(adjust_y), i)

                if self.is_valid(new_y, new_x):
                    still_valid = True

                    for j in range(0, 360, 45):
                        angle = j if j <= 180 else -360 + j
                        for k in range(1, self.action_padding):
                            test_y = new_y + int(k * (np.sin(np.deg2rad(angle))))
                            test_x = new_x + int(k * (np.cos(np.deg2rad(angle))))
                            #rospy.loginfo("LawnbotProblem:actions:Testing Action: y %s and x %s", test_y, test_x)
                            if not self.is_valid(test_y, test_x):
                                still_valid = False
                                break

                    if still_valid:
                        action = np.array([[adjust_y, adjust_x]], int)
                        #print (str(actions.shape) + " " + str(action.shape))
                        # rospy.loginfo("Actions %s y: %s x: %s for shape: %s", actions, str(state[0] + action[0][0]), str(state[1] + action[0][1]), self.state_space.shape)
                        # self.state_space[state[0] + action[0][0]][state[1] + action[0][1]] = 3
                        # rospy.loginfo("Actions %s concating action %s", actions.shape, action.shape)
                        actions = np.concatenate((actions, action))
                        # rospy.loginfo("Actions %s y: %s x: %s", actions, str(state[0] + action[0][1]), state[1] + action[0][1])
                    else:
                        pass  # rospy.loginfo("Action to close to a wall or something y: %s x: %s", test_y, test_x)
                else:
                    pass  # rospy.loginfo("Action Not Valid y: %s x: %s", new_y, new_x)

        #rospy.loginfo("Actions: %s shape %s", str(actions), actions.shape)
        return actions

    def is_valid(self, new_y, new_x):
        if new_y >= self.state_space.state.shape[0]:
            #rospy.loginfo("is_valid: y is %s while state space y is %s", new_y, self.state_space.state.shape[0])
            return False
        if new_x >= self.state_space.state.shape[1]:
            #rospy.loginfo("is_valid: x is %s while state space x is %s", new_x, self.state_space.state.shape[1])
            return False
        if new_y < 0:
            #rospy.loginfo("is_valid: y is %s it cant be negative", new_y)
            return False
        if new_x < 0:
            #rospy.loginfo("is_valid: x is %s it cant be negative", new_x)
            return False
        if self.state_space.state[new_y][new_x] == 2:
            #rospy.loginfo("is_valid: x is %s y is %s cant be a wall", new_x, new_y)
            return False

        return True

    def result(self, state=np.zeros(2), action=np.zeros(2)):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        # Goes y, x
        # self.state_space[state[0] + action[0]][state[1] + action[1]] = 3
        return (state[0] + action[0], state[1] + action[1])

    def goal_test(self, state=np.zeros(2)):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        # rospy.loginfo("The state space value is: %s", self.state_space.state[state[0]][state[1]])
        try:
            if self.state_space.state[state[0]][state[1]] == self.goal:
                # rospy.loginfo("Reached goal state of: %s", self.state_space[state[0]][state[1]])
                return True
        except IndexError:
            pass  # print("Index Error, not goal")

        return False

    def path_cost(self, c, state1, action, state2):
        """
        Note that this is partially the g() function

        Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

    def gen_h(self, state=np.array([0])):
        goal_found = False
        for i in range(1, 360):
            for j in range(1, self.state_space.state.shape[0] - 1
            if self.state_space.state.shape[0] <= self.state_space.state.shape[1]
            else self.state_space.state.shape[1] - 1, 2):

                y = state[0] + j * np.sin(np.deg2rad(i))
                x = state[1] + j * np.cos(np.deg2rad(i))

                if y < self.state_space.state.shape[0] and x < self.state_space.state.shape[1] and\
                                self.state_space.state[y][x] == self.goal:
                    self.h_location = [y, x]
                    break
            if goal_found:
                break

    def f(self, node=SearchNode([0, 0])):
        h = np.sqrt(np.power(node.state[1] - self.h_location[1], 2) +
                    np.power(node.state[0] - self.h_location[0], 2))
        #rospy.loginfo("LawnBotProblem:f x: %s y: %s h_gen is x: %s y: %s where h is %s",
         #             node.state[1], node.state[0], self.h_location[1], self.h_location[0], h)
        return node.path_cost + h
