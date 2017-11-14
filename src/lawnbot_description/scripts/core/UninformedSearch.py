#!/usr/bin/env python
from SearchNode import SearchNode


class UninformedSearch(object):

    def graph_search(self, problem,frontier=list()):
        # Start the frontier with the starting node
        frontier.append(SearchNode(problem.initial))
        # Set the set to empty
        # the starting set data structure is different from
        # lists in that it does not allow duplicates
        explored = set()

        # While the frontier is not empty
        while frontier.__sizeof__() > 0:
            # Get the first child
            node = frontier.pop()

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
            explored.add(node.state)

            # Add new node to the frontier
            frontier.extend(child for child in node.expand(problem)
                            if child.state not in explored and
                            child not in frontier)

        return None

