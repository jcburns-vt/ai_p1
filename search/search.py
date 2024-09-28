# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    x = 'Stop'

    stack = util.Stack()
    visited = []

    # place start node on the stack
    stack.push((problem.getStartState(), []))

    # traverse the graph until nothing remains on the stack
    while not stack.isEmpty():

        # current node and the cummulative path taken to reach this position
        current_node, path_to_current = stack.pop()

        # mark visited
        visited.append(current_node)

        # check if this node is goal
        if problem.isGoalState(current_node): 
            return path_to_current

        # expand node
        successors = problem.getSuccessors(current_node)

        # check if successors have been visited. if not, add to stack
        for s in successors:

            if s[0] in visited:
                pass

            else:
                # stack each unvisited successor with the path to get there
                stack.push((s[0], path_to_current + [s[1]]))

    return  [x]

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    queue = util.Queue()
    visited = []

    # place start node on the stack
    queue.push((problem.getStartState(), []))

    # traverse the graph until nothing remains on the stack
    while not queue.isEmpty():

        # current node and the cummulative path taken to reach this position
        current_node, path_to_current = queue.pop()

        # check that current node hasnt already been visited before expanding it
        if current_node in visited:
            continue

        # mark visited
        visited.append(current_node)

        # check if this node is goal
        if problem.isGoalState(current_node): 
            return path_to_current

        # expand node
        successors = problem.getSuccessors(current_node)

        # check if successors have been visited. if not, add to queue
        for s in successors:

            if s[0] in visited:
                pass

            else:
                # queue each unvisited successor with the path to get there
                queue.push((s[0], path_to_current + [s[1]]))

    return  ['stop']


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    queue = util.PriorityQueue()
    visited = []
    paths = []

    # place start node on the stack
    queue.push((problem.getStartState(), [], 0), 0)

    # traverse the graph until nothing remains on the stack
    while not queue.isEmpty():

        # current node and the cummulative path taken to reach this position
        current_node, path_to_current, cost = queue.pop()

        # check that current node hasnt already been visited before expanding it
        if current_node in visited:
            continue

        # mark visited
        visited.append(current_node)

        # check if this node is goal
        if problem.isGoalState(current_node): 
            paths.append((path_to_current, cost))
            return path_to_current

        # expand node
        successors = problem.getSuccessors(current_node)

        # check if successors have been visited. if not, add to queue
        for s in successors:

            if s[0] in visited:
                pass

            else:
                # queue each unvisited successor with the path to get there
                queue.push((s[0], path_to_current + [s[1]], cost + s[2]), cost + s[2])

    return ['Stop']


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    queue = util.PriorityQueue()
    visited = []
    paths = []

    # place start node on the stack
    queue.push((problem.getStartState(), [], 0), 0)

    # traverse the graph until nothing remains on the stack
    while not queue.isEmpty():

        # current node and the cummulative path taken to reach this position
        current_node, path_to_current, backward_cost = queue.pop()

        # check that current node hasnt already been visited before expanding it
        if current_node in visited:
            continue

        # mark visited
        visited.append(current_node)

        # check if this node is goal
        if problem.isGoalState(current_node): 
            paths.append((path_to_current, backward_cost))
            return path_to_current

        # expand node
        successors = problem.getSuccessors(current_node)

        # check if successors have been visited. if not, add to queue
        for s in successors:

            if s[0] in visited:
                pass

            else:
                # queue each unvisited successor with the path to get there
                queue.push((s[0], path_to_current + [s[1]], backward_cost + s[2]), backward_cost + s[2] + heuristic(s[0], problem))

    return ['Stop']


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
