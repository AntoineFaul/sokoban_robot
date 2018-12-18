from itertools import permutations
import sys
from implementationSokoban import *

class State:
    def __init__(self, grid):
        self.grid = grid

    def __str__(self):
        result = "Player :\t" + str(self.grid.player) + "\nBoxes :"
        for b in self.grid.diamonds:
            result = result + "\t\t" + str(b) + "\n"
        return result + "*********************\n"

class Graph:
    def __init__( self, boxesNumber, initialState):
        self.boxesNumber = boxesNumber
        self.hashFactor = 0

        self.graph = {}
        self.initialState = initialState
        self.finalStates = {}
        self.reachableFinalStates = {}

        # Array of goal distance heuristic for each goals
        # { goal : {(x,y) : heuristic, ....}, ...}
        self.heuristic_goalDistance = {}

        self.graph[ self.hash( initialState)] = []

    def hash( self, state):
        result = 0
        mult = 1

        if self.initialState.grid.height < 10 and self.initialState.grid.width < 10:
            self.hashFactor = 10
        else:
            self.hashFactor = 100

        for i in reversed( range( self.boxesNumber)):
            (x, y) = state.grid.diamonds[i]
            result = result + mult * ( x + self.hashFactor * y)
            mult = mult * self.hashFactor * self.hashFactor
        (x, y) = state.grid.player
        return result + mult * ( x + self.hashFactor * y)

    # Return the state from its hash
    def getStateFromHash(self, hash):
        # State to return
        resultState = self.initialState

        if self.hashFactor == 10:
            mult = 1
        else:
            mult = 2

        # Coordinate extraction of diamonds (row,collumn)
        for i in reversed(range( self.boxesNumber)):
            resultState.grid.diamonds[i] = (int(str(hash)[mult*(i*2+3): mult*(i*2+4)]), int(str(hash)[mult*(i*2+2):mult*(i*2+3)]))

        # Coordinate extraction of player (row,collumn)
        resultState.grid.player = (int(str(hash)[mult:mult*2]),int(str(hash)[0:mult]))
        return resultState

    def add_state( self, state):
        if not self.hash(state) in self.graph:
            self.graph[ self.hash(state)] = []

    def add_edge( self, stateFrom, stateTo):
        if not self.hash(stateFrom) in self.graph:
            self.graph[ self.hash(stateFrom)] = []
        self.graph[ self.hash(stateFrom)].append(  self.hash(stateTo))

    def calculation_GoalDistance( self):

        tamp = [ self.initialState.grid ]
        grid_calcul = copy.deepcopy(tamp)
        grid_calcul = grid_calcul[0]

        for g in grid_calcul.goals:
            cost_so_far = 0
            open = Queue()
            open.put( g)
            visited = []

            self.heuristic_goalDistance[g] = {}
            self.heuristic_goalDistance[g][g] = cost_so_far

            while not open.empty():
                pos = open.get()
                cost_so_far = self.heuristic_goalDistance[g][pos]

                for n in grid_calcul.neighbors( pos):
                    if n not in self.heuristic_goalDistance[g]:
                        self.heuristic_goalDistance[g][n] = cost_so_far + 1
                    else:
                        if self.heuristic_goalDistance[g][n] > cost_so_far + 1:
                            self.heuristic_goalDistance[g][n] = cost_so_far + 1
                    if n not in visited:
                        open.put(n)
                        visited.append(n)

    def getFinalState(self):
        stateTamp = copy.deepcopy( self.initialState)
        cpt = 0

        for iniState in [list(p) for p in permutations( self.initialState.grid.goals)]:
            stateTamp.grid.diamonds = iniState
            self.finalStates[cpt] = copy.deepcopy(stateTamp)
            cpt+=1

    def checkGoal( self, state):
        neighbors = []
        for d in state.grid.diamonds:
            if d not in state.grid.goals:
                return False
            neighbors = neighbors + state.grid.neighbors( d)

        if state.grid.player in neighbors:
            self.reachableFinalStates[ len(self.reachableFinalStates)] = state
            return True
        return False

    # Based on data from pre-processing Data (i.e calculation_GoalDistance), return the minimum value for the goal distance heuristic
    def get_heuristic_goalDistance( self, stateHash):

        heuristic = []
        state = self.getStateFromHash(stateHash)

        if self.reachableFinalStates:
            for fs in self.reachableFinalStates:
                sum = 0;
                for g in state.grid.goals:
                    # Search the id of diamonds matched to the goal g for this reachableFinalStates
                    id = self.reachableFinalStates[fs].grid.diamonds.index(g)
                    sum += self.heuristic_goalDistance[g][ state.grid.diamonds[id]]
                heuristic.append( sum)
            return min( heuristic)

        for fs in self.finalStates:
            sum = 0;
            for g in state.grid.goals:
                print(state.grid.diamonds[ self.finalStates[fs].grid.diamonds.index(g)])
                sum += self.heuristic_goalDistance[g][ state.grid.diamonds[ self.finalStates[fs].grid.diamonds.index(g)]]
            heuristic.append( sum)
        return min( heuristic)

    # Return True if the input state is a reachable final states
    def isInReachableFinalState( self, state):
        # Check for each reachableFinalStates
        for rs in self.reachableFinalStates:
            boolean = True
            # Check the player position
            if not (state.grid.player == self.reachableFinalStates[rs].grid.player):
                boolean = False
                continue
            # Check for each box
            for d in range( self.boxesNumber):
                if not (state.grid.diamonds[d] == self.reachableFinalStates[rs].grid.diamonds[d]):
                    boolean = False
                    break
            if boolean:
                return True
        return False

    def reconstruct_path(self, came_from, startHash, goalHash):
        currentHash = goalHash
        path = []
        while currentHash != startHash:
            current = self.getStateFromHash( currentHash)
            path.append(current.grid.player)
            currentHash = came_from[currentHash]
        path.append(self.getStateFromHash( startHash).grid.player) # optional
        path.reverse() # optional
        return path

def sokoban_goThroughGrid( sokobanGraph, iteration, **style):
    #BFS search to get all of the states reachable on the map
    open = Queue()
    open.put( copy.deepcopy(sokobanGraph.initialState))
    visited = []

    cpt = 0

    while not open.empty():
        current = open.get()
        currentPlayer = current.grid.player

        # Because there is no OpenList we have to check if the current state had not be visited
        if sokobanGraph.hash(current) in visited:
            continue
        # If the sokoban is solved, nothing will change in the children states execpt the player, to reduce the graph it's better to stop to view this branch
        if sokobanGraph.checkGoal( current):
            if 'Break' in style :
                input()
            continue

        if 'Steps' in style :
            print( current)
        if 'Visited' in style :
            print( visited)

        # Update the sokobanGraph
        sokobanGraph.add_state( current)

        for children in current.grid.neighbors(current.grid.player):

            reachable = True
            childrenState = copy.deepcopy( current)

            # update the player position
            childrenState.grid.player = children

            # if the player has to move a box
            if children in current.grid.diamonds:
                ( x, y) = children
                (xP, yP) = currentPlayer
                if childrenState.grid.push( (x,y), ( 2*x-xP, 2*y-yP)):
                    # Update the box position
                    childrenState.grid.diamonds[ current.grid.diamonds.index(children)] = ( 2*x-xP, 2*y-yP)
                    # Update the graph
                    sokobanGraph.add_edge( current, childrenState)
                else:
                    reachable = False
            else:
                # Update the graph
                sokobanGraph.add_edge( current, childrenState)

            if reachable :
                if sokobanGraph.hash(childrenState) not in visited:
                    if 'Allsteps' in style : print( children)
                    open.put(childrenState)
        visited.append(sokobanGraph.hash(current))

        cpt = cpt+1
        if 'Iterations' in style : print(cpt)
        if iteration!=-1 and cpt > iteration:
            break;

def graph_Astar( graph, **style):

    # Copy Object
    #tamp = [ graph ]
    #graph_calcul = copy.deepcopy(tamp)
    #graph_calcul = graph_calcul[0]

    graph_calcul = copy.deepcopy( graph)

    open = PriorityQueue()
    robot = PriorityQueue()
    open.put(graph_calcul.hash(graph_calcul.initialState), 0)

    came_from = {}
    cost_so_far = {}
    came_from[graph_calcul.hash(graph_calcul.initialState)] = None
    cost_so_far[graph_calcul.hash(graph_calcul.initialState)] = 0

    cpt = 0

    while not open.empty():
        currentHash = open.get()
        current = graph_calcul.getStateFromHash( currentHash)

        if 'Steps' in style :
            print( current)

        # The program solve the Sokoban
        if graph_calcul.isInReachableFinalState(current):
            break

        for nextHash in graph_calcul.graph[ currentHash]:
            new_cost = cost_so_far[ currentHash] + 1
            if nextHash not in cost_so_far or new_cost < cost_so_far[nextHash]:
                cost_so_far[nextHash] = new_cost
                priority = new_cost + graph_calcul.get_heuristic_goalDistance(nextHash)
                if 'AllSteps' in style :
                    print( f"{nextHash} heuristic : {graph.get_heuristic_goalDistance(nextHash)} prio : {priority}")
                open.put( nextHash, priority)
                came_from[nextHash] = currentHash

        previous = current

        cpt+=1
        if 'Iterations' in style : print( f"Astar : {cpt}")

    if graph_calcul.isInReachableFinalState(current):
        print( f"FIND : \n{current}")
        return came_from, cost_so_far, currentHash
    else:
        return None

##################################################################


#grid = sokoban_readMap("map/test.txt")
#grid = sokoban_readMap("map/map2017.txt")
#grid = sokoban_readMap("map/map2018.txt")
#grid = sokoban_readMap("map/testMapEasy.txt")
#grid = sokoban_readMap("map/testMapEasy+.txt")
grid = sokoban_readMap("map/testMapNotEasy.txt")
grid.find_deadBlock(steps=[])

sokoban_printMAP( grid, width=2, title="deadBlock and freezeBlock detection", start=grid.player, goal=grid.goals, diamonds=grid.diamonds, deadBlocks=grid.deadBlocks, freezeDiamonds=grid.freezeDiamonds)

currentState = State( grid)

sokobanGraph = Graph( len(currentState.grid.diamonds), currentState)
sokobanGraph.getFinalState()
sokobanGraph.calculation_GoalDistance()

for p in sokobanGraph.finalStates:
    print( sokobanGraph.finalStates[p])

print( f"Hash of the initialState {sokobanGraph.hash(sokobanGraph.initialState)}\n{sokobanGraph.initialState}")
print( "Waiting to run the program ...")
input()

sokoban_goThroughGrid( sokobanGraph, -1, Iterations=[])#, Break=[])#,Steps=[], Allsteps=[])#, #, Visited=[])

#sokobanGraph.getReachableFinalState()

print( "***** END of the search *****")
for p in sokobanGraph.reachableFinalStates:
    print( sokobanGraph.reachableFinalStates[p])

print("*********** A star ***********")
print(sokobanGraph.hash( sokobanGraph.initialState))
came_from, cost_so_far, finalHash = graph_Astar( sokobanGraph)#, Iterations=[]) #Steps=[], AllSteps=[])

print(sokobanGraph.hash( sokobanGraph.initialState))
path= sokobanGraph.reconstruct_path(came_from, startHash=sokobanGraph.hash( sokobanGraph.initialState), goalHash=finalHash)

print( path)
