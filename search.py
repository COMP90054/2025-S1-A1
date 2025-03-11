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
from game import Grid
from searchProblems import nullHeuristic, PositionSearchProblem

### You might need to use
from copy import deepcopy

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    util.raiseNotDefined()


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    myPQ = util.PriorityQueue()
    startState = problem.getStartState()
    startNode = (startState, 0, [])
    myPQ.push(startNode, heuristic(startState, problem))
    best_g = dict()
    while not myPQ.isEmpty():
        node = myPQ.pop()
        state, cost, path = node
        if (not state in best_g) or (cost < best_g[state]):
            best_g[state] = cost
            if problem.isGoalState(state):
                return path
            for succ in problem.getSuccessors(state):
                succState, succAction, succCost = succ
                new_cost = cost + succCost
                newNode = (succState, new_cost, path + [succAction])
                myPQ.push(newNode, heuristic(succState, problem) + new_cost)

    return None  # Goal not found



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

    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    util.raiseNotDefined()


def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    "*** YOUR CODE HERE for TASK1 ***"
    util.raiseNotDefined()
 
def lrtaStarInitial(problem, heuristic=nullHeuristic):
    return lrtaStarTrial(problem, LearningHeuristic(heuristic, problem))

def lrtakStarInitial(problem, heuristic=nullHeuristic):
    return lrtakStarTrial(problem, LearningHeuristic(heuristic, problem), k=9)

class LearningHeuristic:
    """
    This class is designed to help simplify the implementation of LTRA* 
    and LRTA*k algorithms. It allows a single transparent interface to 
    the heuristic function and the updated heuristic cache, without 
    having to initialise the cache for all possible states at the start 
    of the algorithm.

    It also allows easy checking of whether the cache has been updated
    since the last search iteration, so that the search can be stopped
    when the heuristic values have converged.

    This class is not necessary for the implementation of the algorithms,
    but it is recommended to use it to simplify the code.
    """
    def __init__(self,heuristic, problem):
        self.heuristic = heuristic
        self.problem = problem
        self.heuristicCache = {}
        self.updated = False
    
    def getHeuristic(self,state):
        if not state in self.heuristicCache:
            self.heuristicCache[state] = self.heuristic(state, self.problem)
        return self.heuristicCache[state]
    
    def setHeuristic(self,state,h):
        self.heuristicCache[state] = h
        self.updated = True

    def clearUpdates(self):
        self.updated = False

    def wasUpdated(self):
        return self.updated

def lrtaStarSearch(problem, heuristic=nullHeuristic):
    learningHeuristic = LearningHeuristic(heuristic, problem)
    "*** YOUR CODE HERE for TASK2 ***"
    util.raiseNotDefined()

def lrtaStarTrial(problem, learningHeuristic):
    "*** YOUR CODE HERE for TASK2 ***"
    util.raiseNotDefined()
    


def lrtakStarSearch(problem, heuristic=nullHeuristic, k=9):
    learningHeuristic = LearningHeuristic(heuristic, problem)
    "*** YOUR CODE HERE for TASK3 ***"
    util.raiseNotDefined()
  
def lrtakStarTrial(problem, learningHeuristic, k):
    "*** YOUR CODE HERE for TASK3 ***"
    util.raiseNotDefined()
   


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
lrta = lrtaStarSearch
lrtak = lrtakStarSearch
lrtaTrial = lrtaStarInitial
lrtakTrial = lrtakStarInitial
