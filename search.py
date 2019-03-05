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
    #print("Start:", problem.getStartState())
    #print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
#    print("call dfs")
    explored=set()#the explored position or dot
    fringes=util.Stack()# store the latest pushed route
    action_=util.Stack()
    parent={}#key is the child node and value is the parent value
    init_state=problem.getSuccessors(problem.getStartState())
    start_pos=problem.getStartState()
    #print(start_pos)
    #if(problem.isGoalState(start_pos)):
    #    return start_pos
    #else:
    explored.add(start_pos)#have explored the dot
    for i in range(0,len(init_state)):
        parent[init_state[i][0]]=start_pos
        route=[start_pos,init_state[i][0]]
        fringes.push(route)#add the route to the fringes
        action_init=[init_state[i][1]]
        action_.push(action_init)
    #print the child-parent relation
    #for key in parent:
    #    print(parent[key])
    flag=False
    while(not fringes.isEmpty()):
        route=fringes.pop()
        action_direction=action_.pop()
        #print(type(action_direction))
        next_to_expand=route[len(route)-1]
    #    print("next to expand is: ",next_to_expand)
        if(problem.isGoalState(next_to_expand)):
            flag=True
            #print("we found the route")
            #print(route)
            #print("the action_direction",type(action_direction))
            return action_direction
        elif(next_to_expand not in explored):
            explored.add(next_to_expand)
            curr_child=problem.getSuccessors(next_to_expand)
            #print("the next succssor to expand is:",next_to_expand)
            #print(type(problem.getSuccessors(next_to_expand)))
            if(len(curr_child)!=0):#the node does have the child node to expand
                for i in range(0,len(curr_child)):
            #        print("before added, the route: ",route)
                    if(curr_child[i][0] not in explored):
                        #print("the current node's child is ",curr_child[i][0])
                        #print("the current direction is: ",curr_child[i][1])
                        #parent[curr_child[i][0]]=next_to_expand
                        route_=route.copy()
                        route_.append(curr_child[i][0])
                        action_cp=action_direction.copy()
                        action_cp.append(curr_child[i][1])
                        #print("the new route is: ",route_)
                        #print("the new direction is: ",action_cp)
                        fringes.push(route_)
                        action_.push(action_cp)
            else:#the next_to_expand_node doesn't have a child node, and meanwhile it is not the goal then just go to the next route to explored
                continue
        else:
            print("oops we seems to reach the point that previously explored ")
            #print(next_to_expand)
            continue#if the next node to expand has already been expanded, do nothing just
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    explored=set()#the explored position or dot
    fringes=util.Queue()# store the latest pushed route
    action_=util.Queue()
    parent={}#key is the child node and value is the parent value
    init_state=problem.getSuccessors(problem.getStartState())
    start_pos=problem.getStartState()
    if(problem.isGoalState(start_pos)):
        return start_pos
    else:
        explored.add(start_pos)#have explored the dot
    for i in range(0,len(init_state)):
        parent[init_state[i][0]]=start_pos
        route=[start_pos,init_state[i][0]]
        fringes.push(route)#add the route to the fringes
        action_init=[init_state[i][1]]
        action_.push(action_init)
    #print the child-parent relation
    #for key in parent:
    #    print(parent[key])
    flag=False
    while(not fringes.isEmpty()):
        route=fringes.pop()
        action_direction=action_.pop()
        #print(type(action_direction))
        next_to_expand=route[len(route)-1]
        #print("next to expand is: ",next_to_expand)
        if(problem.isGoalState(next_to_expand)):
            flag=True
            #print("we found the route")
            #print(route)
            #print(type(route))
            return action_direction
        elif(next_to_expand not in explored):
            explored.add(next_to_expand)
            curr_child=problem.getSuccessors(next_to_expand)
            #print("the next succssor to expand is:",next_to_expand)
            if(len(curr_child)!=0):#the node does have the child node to expand
                for i in range(0,len(curr_child)):
                    #print("before added, the route: ",route)
                    if(curr_child[i][0] not in explored):
                        #print("the current node's child is ",curr_child[i][0])
                        #print("the current direction is: ",curr_child[i][1])
                        route_=route.copy()
                        route_.append(curr_child[i][0])
                        action_cp=action_direction.copy()
                        action_cp.append(curr_child[i][1])
                        #print("the new route is: ",route_)
                        #print("the new direction is: ",action_cp)
                        fringes.push(route_)
                        action_.push(action_cp)
                    else:
                        if(fringes.isEmpty()):
                            #print("zero!")
                            if(problem.isGoalState(curr_child[i][0])):
                                return action_direction
            else:#the next_to_expand_node doesn't have a child node, and meanwhile it is not the goal then just go to the next route to explored
                continue
        else:
        #    print("oops we seems to reach the point that previously explored ")
        #    print(next_to_expand)
            continue#if the next node to expand has already been expanded, do nothing just
            #return
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    explored=set()#the explored position or dot
    fringes=util.PriorityQueue()# store the latest pushed route
    action_=util.PriorityQueue()
    priority_store=util.PriorityQueue()
    parent={}#key is the child node and value is the parent value
    init_state=problem.getSuccessors(problem.getStartState())
    start_pos=problem.getStartState()
    if(problem.isGoalState(start_pos)):
        return start_pos
    else:
        explored.add(start_pos)#have explored the dot
    for i in range(0,len(init_state)):
        parent[init_state[i][0]]=start_pos
        route=[start_pos,init_state[i][0]]
        fringes.push(route,init_state[i][2])#add the route and the initial cost to the fringes
        priority_store.push(init_state[i][2],init_state[i][2])
        action_init=[init_state[i][1]]
        action_.push(action_init,init_state[i][2])#also add the cost to the action
    flag=False
    while(not fringes.isEmpty()):
        route=fringes.pop()
        action_direction=action_.pop()
        curr_priority=priority_store.pop()
        #print(type(action_direction))
        next_to_expand=route[len(route)-1]
    #    print("next to expand is: ",next_to_expand)
        if(problem.isGoalState(next_to_expand)):
            flag=True
            #print("we found the route")
            #print(route)
            #print(type(route))
            return action_direction
        elif(next_to_expand not in explored):
            explored.add(next_to_expand)
            curr_child=problem.getSuccessors(next_to_expand)
            #print("the next succssor to expand is:",next_to_expand)
            #print(type(problem.getSuccessors(next_to_expand)))
            if(len(curr_child)!=0):#the node does have the child node to expand
                for i in range(0,len(curr_child)):
            #        print("before added, the route: ",route)
                    if(curr_child[i][0] not in explored):
                        #print("the current node's child is ",curr_child[i][0])
                        #print("the current direction is: ",curr_child[i][1])
                        #parent[curr_child[i][0]]=next_to_expand
                        prior_update=curr_priority+curr_child[i][2]
                        route_=route.copy()
                        route_.append(curr_child[i][0])
                        action_cp=action_direction.copy()
                        action_cp.append(curr_child[i][1])
                        #print("the new route is: ",route_)
                        #print("the new direction is: ",action_cp)
                        fringes.update(route_,prior_update)
                        action_.update(action_cp,prior_update)
                        priority_store.push(prior_update,prior_update)
            else:#the next_to_expand_node doesn't have a child node, and meanwhile it is not the goal then just go to the next route to explored
                continue
        else:
        #    print("oops we seems to reach the point that previously explored ")
        #    print(next_to_expand)
            continue#if the next node to expand has already been expanded, do nothing just
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    explored=set()#the explored position or dot
    fringes=util.PriorityQueue()# store the latest pushed route
    action_=util.PriorityQueue()
    priority_store=util.PriorityQueue()
    g_cost_store=util.PriorityQueue()#store the gx cost of route
    #parent={}#key is the child node and value is the parent value
    init_state=problem.getSuccessors(problem.getStartState())
    start_pos=problem.getStartState()
    if(problem.isGoalState(start_pos)):
        return start_pos
    else:
        explored.add(start_pos)#have explored the dot
    root_cost=heuristic(start_pos,problem)
    for i in range(0,len(init_state)):
        #parent[init_state[i][0]]=start_pos
        route=[start_pos,init_state[i][0]]
        h_cost=heuristic(init_state[i][0],problem)
        fcost=h_cost+init_state[i][2]
        fringes.push(route,fcost)#add the route and the initial fcost to the fringes
        priority_store.push(fcost,fcost)
        g_cost_store.push(init_state[i][2],fcost)
        action_init=[init_state[i][1]]
        action_.push(action_init,fcost)#also add the cost to the action
    #    print("push fringes, and fcost:",route,fcost)
    flag=False
    while(not fringes.isEmpty()):
        route=fringes.pop()
        action_direction=action_.pop()
        curr_priority_gh=priority_store.pop()
        curr_prior_g=g_cost_store.pop()
        next_to_expand=route[len(route)-1]
    #    print("next to expand is: ",next_to_expand)
        if(problem.isGoalState(next_to_expand)):
            flag=True
        #    print("we found the route")
        #    print(route)
            #print(type(route))
            return action_direction
        elif(next_to_expand not in explored):
            explored.add(next_to_expand)
            curr_child=problem.getSuccessors(next_to_expand)
        #    print("the next succssor to expand is:",next_to_expand)
            #print(type(problem.getSuccessors(next_to_expand)))
            if(len(curr_child)!=0):#the node does have the child node to expand
                for i in range(0,len(curr_child)):
                    #print("before added, the route: ",route)
                    #print("the current node's child is ",curr_child[i][0])
                    if(curr_child[i][0] not in explored):
                        #print("the child node has not been expanded: ",curr_child[i][0])
                        #print("the current direction is: ",curr_child[i][1])
                        #parent[curr_child[i][0]]=next_to_expand
                        h_distance=heuristic(curr_child[i][0],problem)
                        prior_update=curr_prior_g+curr_child[i][2]+h_distance
                        g_cost_update=curr_prior_g+curr_child[i][2]
                        #print("new hx=: ",h_distance)
                        #print("new actual cost=: ",prior_update)
                        route_=route.copy()
                        route_.append(curr_child[i][0])
                        action_cp=action_direction.copy()
                        action_cp.append(curr_child[i][1])
                        #print("the new route is: ",route_)
                        #print("the new direction is: ",action_cp)
                        fringes.update(route_,prior_update)
                        action_.update(action_cp,prior_update)
                        priority_store.push(prior_update,prior_update)
                        g_cost_store.push(g_cost_update,prior_update)
            else:#the next_to_expand_node doesn't have a child node, and meanwhile it is not the goal then just go to the next route to explored
                continue
        else:
            #print("oops we seems to reach the point that previously explored ")
            #print(next_to_expand)
            continue#if the next node to expand has already been expanded, do nothing just
    #util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
