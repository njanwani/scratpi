#!/usr/bin/env python3
#
#   prmtools.py
#
#   The defines the Node class, from which to build the graphs, as
#   well as an A* algorithm to find a path through the graph.
#
#   To use:
#     from prmtools import Node, Astar
#
import bisect
import math
import numpy as np

D = 0.05 # 0.15
D_WALL = 0.08
GRID_X = -3.8100
GRID_Y = -3.8100
RES = 0.0254

class State:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def Distance(self, state):
        return ((self.y - state.y)**2 + (self.x - state.x)**2)**0.5


#
#   Node class upon which to build the graph (roadmap) and which
#   supports the A* search tree.
#
class Node:
    def __init__(self, state, map):
        # Save the state matching this node.
        self.state = state

        self.map = map

        # Edges used for the graph structure (roadmap).
        self.children = []
        self.parents  = []

        # Status, edge, and costs for the A* search tree.
        self.seen        = False
        self.done        = False
        self.treeparent  = []
        self.costToReach = 0
        self.costToGoEst = math.inf
        self.cost        = self.costToReach + self.costToGoEst

    # Define the "less-than" to enable sorting by cost in A*.
    def __lt__(self, other):
        return self.cost < other.cost

    # Distance to another node, for A*, using the state distance.
    def Distance(self, other):
        return self.state.Distance(other.state)

    def connectsTo(self, node):
        return self.connectsToUV(node)[0]

    def connectsToUV(self, node):
        x_A = self.state.x
        y_A = self.state.y

        x_B = node.state.x
        y_B = node.state.y

        if not self.connectsToHelper(x_A, y_A):
            return False, (int((x_A - GRID_X) / RES), int((y_A - GRID_Y) / RES))
        elif not self.connectsToHelper(x_B, y_B):
            return False, (int((x_B - GRID_X) / RES), int((y_B - GRID_Y) / RES))
        else:
            dist = ((x_B - x_A)**2 + (y_B - y_A)**2)**0.5
            x = np.linspace(x_A, x_B, num=dist / D)
            y = np.linspace(y_A, y_B, num=dist / D)
            for i in range(len(x)):
                if not self.connectsToHelper(x[i], y[i]):
                    return False, (int((x[i] - GRID_X) / RES), int((y[i] - GRID_Y) / RES))
            
            return True, 




    def connectsToHelper(self, x, y):
        u = int((x - GRID_X) / RES)
        v = int((y - GRID_Y) / RES)

        p_uv = self.map[v, u]

        p_x = (float(p_uv[0])+0.5) * RES + GRID_X
        p_y = (float(p_uv[1])+0.5) * RES + GRID_Y

        d = math.sqrt((x - p_x)**2 + (y - p_y)**2)
        return d > D_WALL

    def freespaceCost(self):
        u = int((self.state.x - GRID_X) / RES)
        v = int((self.state.y - GRID_Y) / RES)
        denom = np.linalg.norm([u - self.map[(v,u)][0],
                                v - self.map[(v,u)][1]])
        if denom == 0:
            denom = 0.1
        return 1.0 / denom




def verifyPath(path, wallptmap):
    new_path = []
    for node in path:
        new_path.append(Node(node.state, wallptmap))

    for i in range(len(new_path) - 1):
        # print('checking...')
        ret = new_path[i].connectsToUV(new_path[i + 1])
        if not ret[0]:
            return ret
    return True, None


#
#   A* Planning Algorithm
#
def AStar(nodeList, start, goal):
    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Clear the search tree (for repeated searches).
    for node in nodeList:
        node.seen = False
        node.done = False

    # Begin with the start state on-deck.
    start.done        = False
    start.seen        = True
    start.treeparent  = None
    start.costToReach = 0
    start.costToGoEst = start.Distance(goal)
    start.cost        = start.costToReach + start.costToGoEst
    bisect.insort(onDeck, start)

    i = 0
    # Continually expand/build the search tree.
    while True:
        print("iter ", i)
        i += 1
        # Grab the next node (first on deck).
        node = onDeck.pop(0)


        # Add the children to the on-deck queue (or update)
        for child in nodeList:
            # Skip if already done.
            if child.done or node == child or not child.connectsTo(node): # child.Distance(node) > 0.3:
                continue

            # Compute the cost to reach the child via this new path.
            costToReach = node.costToReach + node.Distance(child)

            # Just add to on-deck if not yet seen (in correct order).
            if not child.seen:
                child.seen        = True
                child.treeparent  = node
                child.costToReach = costToReach
                child.costToGoEst = child.Distance(goal)
                child.cost        = child.costToReach + child.costToGoEst + child.freespaceCost()
                bisect.insort(onDeck, child)
                continue

            # Skip if the previous cost was better!
            if child.costToReach <= costToReach:
                continue

            # Update the child's connection and resort the on-deck queue.
            child.treeparent  = node
            child.costToReach = costToReach
            child.cost        = child.costToReach + child.costToGoEst
            onDeck.remove(child)
            bisect.insort(onDeck, child)

        # Declare this node done.
        node.done = True

        # Check whether we have processed the goal (now done).
        if (goal.done):
            break

        # Also make sure we still have something to look at!
        if not (len(onDeck) > 0):
            return []

    # Build the path.
    path = [goal]
    while path[0].treeparent is not None:
        path.insert(0, path[0].treeparent)

    # Return the path.
    return path
