#!/usr/bin/env python
from SearchNode import SearchNode

import rospy

from State import State
from Queues import PriorityQueue
from Queues import FIFOQueue
from util import memoize


class UninformedSearch(object):

    def graph_search(self, problem,frontier=list()):
        # Start the frontier with the starting node
        frontier.append(SearchNode(problem.initial))
        # Set the set to empty
        # the starting set data structure is different from
        # lists in that it does not allow duplicates
        explored = set()

        # While the frontier is not empty
        while frontier:
            # Get the first child
            node = frontier.pop()
            #rospy.loginfo("Going to frontier of: %s", frontier.__str__())

            # Original code had the goal test being the state
            # I just want the position
            # the goal test will not use the state as the
            # goal test,instead it will use the current location
            # and determineif that location is explored.
            # if not, then it has reached its goal!
            if problem.goal_test(node.state):
                # This node should contain an x y
                # sequence for the robot to follow
                return node

            # If the node is not the goal state then
            # add it to the explored set
            explored.add(tuple(node.state))

            while State.lock:
                print ("Waiting...")
                pass

            # Add new node to the frontier
            frontier.extend(child for child in node.expand(problem)
                            if tuple(child.state) not in explored and
                            child not in frontier)

        return None

    def best_first_graph_search(self, problem, f):
        """Search the nodes with the lowest f scores first.
        You specify the function f(node) that you want to minimize; for example,
        if f is a heuristic estimate to the goal, then we have greedy best
        first search; if f is node.depth then we have breadth-first search.
        There is a subtlety: the line "f = memoize(f, 'f')" means that the f
        values will be cached on the nodes as they are computed. So after doing
        a best first search you can examine the f values of the path returned."""
        f = memoize(f, 'f')
        # Set starting node
        node = SearchNode(problem.initial)
        # If the goal is reached, return the resulting node
        if problem.goal_test(node.state):
            return node

        # Set priority queue to organize nodes
        # in order of lowest f
        frontier = PriorityQueue(min, f)
        # Append the first node
        frontier.append(node)
        # Initialize empty set
        explored = set()
        # While the frontier is not empty
        while frontier:
            # Get the first node with lowest f
            node = frontier.pop()
            # Check if node is goal
            if problem.goal_test(node.state):
                return node
            # Add the state to the explored set
            explored.add(tuple(node.state))
            # For every child in the expanded node
            for child in node.expand(problem):
                # If the child is not a repeat child append it
                if child.state not in explored and child not in frontier:
                    frontier.append(child)
                # If the child is in the frontier
                # This statement basically just filters out children that
                # have the same state but lower path costs
                elif child in frontier:
                    # Select that child
                    incumbent = frontier[child]
                    # If one child is has a lower path cost
                    if f(child) < f(incumbent):
                        # Remove the child that is farther
                        del frontier[incumbent]
                        frontier.append(child)
        return None

