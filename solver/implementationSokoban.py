from implementation  import *
from Astar import *
import re
import turtle
import copy

class sokoban_GridWithWeights(GridWithWeights):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.player = ()
        self.diamonds = []
        self.goals = []

        self.deadBlocks = []
        self.freezeDiamonds = []

    def __copy__(self):
        return type(self)(self.player, self.diamonds, self.goals, self.deadBlocks, self.freezeDiamonds, self.weights, self.width, self.height, self.walls, self.nodes)

    def passable(self, id):
        return super().passable(id) and id not in self.diamonds

    def pull( self, from_id, to_id):
        (x, y) = from_id
        neighbors = [(x+1,y), (x, y-1), (x-1, y), (x, y+1)]
        extend_neighbors = [(x+2,y), (x, y-2), (x-2, y), (x, y+2)]
        if not to_id in neighbors:
            return False
        if to_id in self.walls:
            return False
        if extend_neighbors[ neighbors.index(to_id)] in self.walls:
            return False
        return True

    def push( self, from_id, to_id):
        (x, y) = from_id
        (x_to, y_to) = to_id

        # XOR :     A   B   A^B
        #           0   0    0
        #           1   0    1
        #           0   1    1
        #           1   1    0
        if not( (abs(x-x_to) == 1) ^ (abs(y-y_to) == 1)):
            return False
        if (x_to, y_to) in self.walls:
            return False
        if (x_to, y_to) in self.diamonds:
            return False
        if (x_to, y_to) in self.deadBlocks:
            return False
        return True

    def isReachable( self, to_id, from_id):
        (x, y) = from_id
        (x_to, y_to) = to_id
        path = a_star_search(self, self.player, (x+x-x_to, y+y-y_to))
        if path:
            return True
        else:
            return False

    def find_deadBlock( self, **style):
        for goal in self.goals:
            if 'steps' in style :
                print( f"deadblock detection by starting on {goal}")

            open = Queue()
            open.put(goal)
            visited = []

            while not open.empty():
                current = open.get()

                if 'Allsteps' in style :
                    print( f"Current : {current}")

                for children in self.neighbors(current):
                    if children not in visited:
                        if 'Allsteps' in style : print( children)
                        if self.pull( current, children):
                            open.put(children)
                    visited.append(current)

            if not self.deadBlocks:
                print("plop")
                self.deadBlocks = [x for x in self.nodes if x not in visited]
            else :
                self.deadBlocks = [x for x in self.deadBlocks if x not in visited]

            if 'steps' in style :
                print( f"deadBlocks : {self.deadBlocks}")

    def find_freezeBlocks( self, **style):
        for diamond in self.diamonds:

            if 'steps' in style :
                print( f"diamond : {diamond}")

            (x, y) = diamond
            block_horizontally = True
            block_vertically = True

            neighbors = [(x+1,y), (x, y-1), (x-1, y), (x, y+1)]

            if self.in_bounds((x, y-1)) and self.in_bounds((x, y+1)):
                if self.passable((x, y-1)) and self.passable((x, y+1)):
                    if not((x, y-1) in self.diamonds) and not((x, y+1) in self.diamonds):
                        if not((x, y-1) in self.deadBlocks and (x, y+1) in self.deadBlocks):
                            block_horizontally = False
                            if 'steps' in style : print("No block block_horizontally")

            if self.in_bounds((x-1, y)) and self.in_bounds((x+1, y)):
                if self.passable((x-1, y)) and self.passable((x+1, y)):
                    if not((x-1, y) in self.diamonds) and not((x+1, y) in self.diamonds):
                        if not((x-1, y) in self.deadBlocks and (x+1, y) in self.deadBlocks):
                            block_vertically = False
                            if 'steps' in style : print("No block block_vertically")

            if block_vertically and block_horizontally :
                self.freezeDiamonds.append( diamond)

    def freezeBlock( self, id):
        (x, y) = id
        block_horizontally = True
        block_vertically = True

        neighbors = [(x+1,y), (x, y-1), (x-1, y), (x, y+1)]

        if self.in_bounds((x, y-1)) and self.in_bounds((x, y+1)):
            if self.passable((x, y-1)) and self.passable((x, y+1)):
                if not((x, y-1) in self.diamonds) and not((x, y+1) in self.diamonds):
                    if not((x, y-1) in self.deadBlocks and (x, y+1) in  self.deadBlocks):
                        block_horizontally = False

        if self.in_bounds((x-1, y)) and self.in_bounds((x+1, y)):
            if self.passable((x-1, y)) and self.passable((x+1, y)):
                if not((x-1, y) in self.diamonds) and not((x+1, y) in self.diamonds):
                    if not((x-1, y) in self.deadBlocks and (x+1, y) in self.deadBlocks):
                        block_vertically = False

        return block_vertically and block_horizontally

    def calculation_heuristic( self):
        heuristic = {}

        #for diamond in [x for x in self.diamonds if x not in self.freezeDiamonds]:
        for diamond in self.diamonds:
            heuristic[diamond] = []
            for goal in self.goals:
                heuristic[diamond].append( [goal, heuristicFuntion(diamond, goal)])

        return heuristic

    def sort_heuristic( self, heuristic):
        # Compilation of heuristics
        plan_heurisitic = {}
        for d in [x for x in self.diamonds if x not in self.freezeDiamonds]:
            w = 0
            for h in heuristic[d]:
                #print(f"h : {h}")
                #print(h[g])
                if not plan_heurisitic.get(h[1]):
                    plan_heurisitic.update( {h[1]: [[d, h[0]]]})
                else:
                    plan_heurisitic[h[1]].append([d, h[0]])
                w = w+1
        return plan_heurisitic

def sokoban_readMap( name):

    myFile = open(name, "r")

    lines = myFile.readlines()

    Xsize = int(float(lines[0][:2]))
    Ysize = int(float(lines[0][3:5]))
    DiamondNumber = int(float(lines[0][6:]))

    print(f"Map detected\nInformation [X={Xsize}, Y={Ysize}, DiamondNumber={DiamondNumber}]")

    grid = sokoban_GridWithWeights(Xsize, Ysize)

    for i in range(1, Ysize+1):

        for m in re.finditer(' ', lines[i]):
            grid.walls.append( (i-1, m.start()) )

        for m in re.finditer('X', lines[i]):
            grid.walls.append( (i-1, m.start()) )

        if( len(lines[i])<Xsize):
            for j in range( len(lines[i])-1, Xsize):
                grid.walls.append( (i-1,j) )

        for m in re.finditer('J', lines[i]):
            grid.diamonds.append( (i-1, m.start()))

        for m in re.finditer('G', lines[i]):
            grid.goals.append( (i-1, m.start()))

        if( lines[i].find('M') != -1):
            grid.player = (i-1, lines[i].find('M'))

    for i in range( grid.height):
        for j in range( grid.width):
            if not((i,j) in grid.walls) :
                grid.nodes.append( (i,j))

    sokoban_printMAP(grid, width=2, start=grid.player, goal=grid.goals, diamonds=grid.diamonds)

    myFile.close()

    return grid

def sokoban_printMAP(graph, width=2, **style):


    print("*********************\n")
    if 'title' in style : print( f"{style['title']}\n")

    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (y, x), style, width), end="")
        print()

    print("\n*********************")

def print_heuristic( graph, heuristic):

    print("\nHeuristic : *********\n\t\t", end="")
    for i in range( len(graph.goals)):
        print("  G%d" % (i), end="\t")
    print("\n\t\t", end="")
    for g in graph.goals:
        print("%s" % str(g), end="\t")

    i = 0
    for d in graph.diamonds:
        print(f"\nD{i} {d}", end="\t")
        i = i+1
        for x in heuristic[d]:
            print("   %s" % str(x), end="\t")
        if d in graph.freezeDiamonds:
            print("B", end="")
    print()

def sokoban_a_star_search(graph, start, goal):

    # Copy Object
    tamp = [ graph ]
    graph_calcul = copy.deepcopy(tamp)
    graph_calcul = graph_calcul[0]

    open = PriorityQueue()
    robot = PriorityQueue()
    open.put(start, 0)
    robot.put(graph.start, 0)

    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    graph_calcul.diamonds.remove( start)

    while not open.empty():
        current = open.get()
        graph_calcul.start = robot.get()

        if current == goal:
            break

        for next in graph_calcul.neighbors(current):
            new_cost = cost_so_far[current] + graph_calcul.cost(current, next)
            if next == goal or not graph_calcul.freezeBlock(next):
                if graph_calcul.push( current, next):
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + heuristicFuntion(goal, next)
                        open.put(next, priority)
                        robot.put(current, priority)
                        came_from[next] = current

        previous = current

    if current == goal :
        return came_from, cost_so_far
    else:
        return None

def heuristicFuntion(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path
