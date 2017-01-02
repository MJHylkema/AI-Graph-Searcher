from search import *
import math
import queue

class MapGraph(Graph):
    
    def __init__(self, graph):
        self.graph = graph
        self.x_nodes = []
        self.s_nodes = []
        self.g_nodes = []
        self.positions = []
        self.graph_points = []
        
        #create an array of points of the graph
        self.graph_points = self.graph.strip().splitlines()
	
        for i in range(len(self.graph_points)):
            self.graph_points[i] = list(self.graph_points[i].strip())
        
        #fill the lists with corresponding graph positions
        for row in range(len(self.graph_points)):
            for col in range(len(self.graph_points[0])):
                #Add all points positions to the position list
                if self.graph_points[row][col] in [" ", "S", "G"]:
                    self.positions.append((row, col))
                #Add points that are a goal, start or wall node to their lists
                if self.graph_points[row][col] in ["G"]:
                    self.g_nodes.append((row, col))
                elif self.graph_points[row][col] in ["S"]:
                    self.s_nodes.append((row, col))
                elif self.graph_points[row][col] in ["X"]:
                    self.x_nodes.append((row, col))
        

    def is_goal(self, node):
        """Returns true if the given node is a goal state."""
        return (node in self.g_nodes)

    def starting_nodes(self):
        """Return a sequence of starting nodes. Often there is only one
        starting node but even then the function returns a sequence
        with one element. It can be implemented as an iterator."""
        ##for node in self.s_nodes:
        ##    yield node   
        return self.s_nodes
        
    def outgoing_arcs(self, tail):
        """Given a node it returns a sequence of arcs (Arc objects)
        which correspond to the actions that can be taken in that
        state (node)."""
        label = ''; head = ''; cost = 0
        
        #A list of all possible positions of nodes surounding the tail
        arcs = [('N' , -1, 0), ('NE', -1, 1), ('E' ,  0, 1), ('SE',  1, 1),
               ('S' ,  1, 0), ('SW',  1, -1), ('W' ,  0, -1), ('NW', -1, -1)]        
        
        #Produce an arc for any node around the tail that is available                                            
        for i in range(len(arcs)):
            temp_node = (tail[0]+arcs[i][1],tail[1]+arcs[i][2])
            if(temp_node in self.positions and temp_node not in self.x_nodes):
                head = (temp_node); label = arcs[i][0]; cost = 1
                yield Arc(tail, head, label, cost)
        
        
    def estimated_cost_to_goal(self, node):
        """Return the estimated cost to a goal node from the given
        state. This function is usually implemented when there is a
        single goal state. The function is used as a heuristic in
        search. The implementation should make sure that the heuristic
        meets the required criteria for heuristics."""
	
	#Use Euclidean distance to underestimate the distance of node to goal
        goal = self.g_nodes[0]
        row = abs(goal[0]-node[0])
        col = abs(goal[1]-node[1])
        dist = 0
        if(row > col):
            dist = col + (row-col)
        elif(col > row):
            dist = row + (col-row)
        else:
            dist = row
	    
        return dist
	
class AStarFrontier(Frontier):
    def __init__(self, graph):
        self.graph = graph
        self.visited = []
        self.container = []
        
    def add(self, path):
	#If the head of the path has not been visited before, append to container
        if(path[-1].head not in self.visited):
            self.container.append(path)
    
    def __iter__(self):
	#While the container contains atleast one path iterate.
        while len(self.container) > 0:
            lowest_cost = 100
            pos = 0
	    
	    #Determine the total cost of each frontier node by looking at the paths
            for path in self.container:
                path_cost = 0
		#Determines the path cost by adding the cost of every arc in this path
                path_cost = sum(path[arc].cost for arc in range(len(path)))
		#Determines the cost to goal using the heuristic from MapGraph
                heuristic_cost = self.graph.estimated_cost_to_goal(path[-1].head)
		#Adds the path cost and heuristic cost to get a total cost
                total_cost = (path_cost + heuristic_cost)	
                #print("Cost: ", path[-1].head, " = ", total_cost)
		#If the total cost is lower than the current lowest cost: 
                if total_cost < lowest_cost:
		    #Set the container position to the lowest cost path
                    pos = self.container.index(path)
		    #Re-assign the lowest cost to be the new lowest cost
                    lowest_cost = total_cost
            #Pop the lowest cost path head from the container        
            visited_node = self.container.pop(pos)
	    #Add path head to the visited list
            if(visited_node[-1].head not in self.visited):
                self.visited.append(visited_node[-1].head)
		#Yields the lowest cost path
                yield visited_node        

class LCFSFrontier(Frontier):
    def __init__(self):
        self.container = queue.Queue()
        self.visited = []
        
    def add(self, path):
	#If the head of the path has not been visited before, append to container
        if(path[-1].head not in self.visited):
            self.container.put(path)
            
    
    def __iter__(self):
	#While the container is not empty iterate.
        while not(self.container.empty()):
	    #Remove the path at the head of the queue
            visited_node = self.container.get()
	    #If the head of the path hasnt already been visited, add to visit list
            if(visited_node[-1].head not in self.visited):
                self.visited.append(visited_node[-1].head)
            yield visited_node


def print_map(graph, frontier, solution):
    #A temporary plot of the graph points from MapGraph
    plot = graph.graph_points    
    
    #Plot all nodes from the visited list as "."
    for node in frontier.visited:
        if(plot[node[0]][node[1]] == " "):
            plot[node[0]][node[1]] = "."
    
    #If a solution is present, replace the path nodes with "*"	
    if(solution):
        for arc in solution:
            if(arc.head != None and plot[arc.head[0]][arc.head[1]] != "S" 
	       and plot[arc.head[0]][arc.head[1]] != "G"):
                plot[arc.head[0]][arc.head[1]] = "*"
    
    #Plot the graph
    for row in range(len(plot)):
        for col in range(len(plot[0])):
            if(col == len(plot[0])-1):
                print(plot[row][col], "\n", end="")
            else:
                print(plot[row][col], end="")
	
def main():   
    map_str = """\
    +-------+
    |     XG|
    |X XXX  |
    | S     |
    +-------+
    """
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)
    print("-------------------------------------------------")
    map_str = """\
    +--+
    |GS|
    +--+
    """
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)
    print("-------------------------------------------------")
    map_str = """\
    +----+
    |    |
    | SX |
    | X G|
    +----+
    """
    
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)  
    print("-------------------------------------------------")
    map_str = """\
    +------------+
    |            |
    |            |
    |            |
    |    S       |
    |            |
    |            |
    | G          |
    |            |
    +------------+
    """
    
    map_graph = MapGraph(map_str)
    frontier = LCFSFrontier()
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)    
    print("-------------------------------------------------")
    map_str = """\
    +------------+
    |            |
    |            |
    |            |
    |    S       |
    |            |
    |            |
    | G          |
    |            |
    +------------+
    """
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)  
    print("-------------------------------------------------")
    map_str = """\
    +---------------+
    |    G          |
    |XXXXXXXXXXXX   |
    |           X   |
    |  XXXXXX   X   |
    |  X S  X   X   |
    |  X        X   |
    |  XXXXXXXXXX   |
    |               |
    +---------------+
    """
    
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)   
    print("-------------------------------------------------")
    map_str = """\
    +---------+
    |         |
    |    G    |
    |         |
    +---------+
    """
    
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)    
    print("-------------------------------------------------")
    map_str = """\
    +-------------+
    |         G   |
    | S           |
    |         S   |
    +-------------+
    """
    
    map_graph = MapGraph(map_str)
    frontier = AStarFrontier(map_graph)
    solution = generic_search(map_graph, frontier)
    print_map(map_graph, frontier, solution)    
if __name__ == "__main__":
    main()