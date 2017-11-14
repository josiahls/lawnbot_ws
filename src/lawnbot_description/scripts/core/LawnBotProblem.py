#!/usr/bin/env python
import numpy as np


class LawnBotProblem(object):

    """The abstract class for a formal problem.  You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=0, state_space=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal
        self.state_space = state_space

    def actions(self, state=np.zeros(2)):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        actions = np.zeros(1,2)
        for i in range(1,360):
            new_x = int(state[0] + np.cos(np.deg2rad(i)))
            new_y = int(state[1] + np.sin(np.deg2rad(i)))

            adjust_x = int(np.sin(np.deg2rad(i)))
            adjust_y = int(np.cos(np.deg2rad(i)))

            try:
                if self.state_space[new_y][new_x] != 2:
                    np.concatenate((actions, (adjust_y, adjust_x)))
            except IndexError:
                pass

        return actions




    def result(self, state=np.zeros(2), action=np.zeros(2)):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        self.state_space[state[0] + action[0]][state[1] + action[1]] = 3
        return (state[0] + action[0], state[1] + action[1])

    def goal_test(self, state=np.zeros(2)):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if self.state_space[state[0]][state[1]] == self.goal:
            return True

        return False





    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError