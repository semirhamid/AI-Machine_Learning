# Adjacency list
# each node have node and neighbours

from collections import defaultdict, deque
from math import sqrt
from queue import PriorityQueue

# we used defaultdict not to wirte many if else conditions(more readable code)       
class Graph:
    def __init__(self) -> None:
        self.graph = defaultdict(list)

        # latitude and longtiude location for each cities
        self.location = {   "Arad": [46.1848, 21.3120],
                            "Bucharest": [44.4268, 26.1025],
                            "Craiova": [44.3302, 23.7949],
                            "Drobeta": [44.6300, 22.6572],
                            "Giurgiu": [43.9000, 25.9667],
                            "Iasi": [47.1585, 27.6014],
                            "Oradea": [47.0722, 21.9217],
                            "Neamt": [46.9333, 26.3333],
                            "Pitesti": [44.8563, 24.8696],
                            "Rimnicu Vilcea": [45.1000, 24.3667],
                            "Resita": [45.3000, 21.8833],
                            "Sfantu Gheorghe": [45.8667, 25.7833],
                            "Sibiu": [45.8000, 24.1500],
                            "Zerind": [46.6167, 21.5167],
                            "Timisoara": [45.7489, 21.2087],
                            "Lugoj": [45.6864, 21.9039],
                            "Mehadia": [44.9000, 22.3667],
                            "Fagaras": [45.8428, 24.9722],
                            "Urziceni": [44.7167, 26.6333],
                            "Hirsova": [44.6833, 27.9500],
                            "Vaslui": [46.6333, 27.7333],
                            "Eforie": [44.0667, 28.6333]
                        }
        

    # create node with value and edsges list
    def createNode(self, node,neighbours = []):
       self.graph[node] = neighbours
    
    # add new node with node and neighbours list(set to empty list if neighbours aren't provided)
    def addNode(self, node, neighbours = []):
        self.createNode(node, neighbours)

    # remove node
    def removeNode(self, node):
        self.graph.pop(node) 
    
    # add neighbours and make connection between them
    def addNeighbour(self, node, neighbour, weight = 0):
        self.graph[node].append((neighbour, weight ))
        
        if (node, weight) in self.graph[neighbour]:
            return
        
        self.graph[neighbour].append((node, weight))
    
    # add neighbours to a node

    def addNeighbours(self, node, neighbours = []):
        self.graph[node].extend(neighbours)

    # remove neighbours(connections) of a node O(n) time complexity
    def removeNeighbour(self, node, neighbour, weight):
        if node in self.graph:

            try:
                self.graph[node].remove((neighbour, weight))
                self.graph[neighbour].remove((node, weight))

            except ValueError:
                print("neighbour doesn't exist")

    # get all neighbours(connections) of a node
    def getConnections(self, node):
        return self.graph[node]

    # get the entire graph
    def printGraph(self):
        return self.graph
    
    #check if the node exists in the graph
    def isNodeExists(self, node):
        return node in self.graph
    



    # BFS
    def BFS(self, start):
        visted = {start}
        queue = deque([start])

        while queue:
            curr = queue.popleft()
            print(curr)
            for neighbour, weight in self.graph[curr]:
                if neighbour not in visted:
                    visted.add(neighbour)
                    queue.append(neighbour)

    def DFS(self, start, visted = set()):
        if start not in visted:
            visted.add(start)
            print(visted)
            for neighbour in self.graph[start]:
                self.DFS(neighbour, visted)



    def UCS(self, start, goal, path = [], cost = 0):
            queue = PriorityQueue()
            queue.put((cost, start, path))

            visted = set()

            while not  queue.empty():
                cost, node, path = queue.get()

                if goal == node:
                    return (cost, path + [node])
                
                visted.add(node)

                for neighbour, edge_cost in self.graph[node]:
                    if neighbour in visted:
                        continue
                    
                    total_cost = cost + edge_cost
                    queue.put((total_cost, neighbour, path + [node]))

            return "No path to reach to {goal}".format(goal = goal)


    def iterativeDeepening(self,start, goal, maxDepth):

        def DLS(currNode, currDepth, path, visted ):

            if currNode == goal:
                path.append(currNode)
                return True
            
            if currDepth <= 0:
                return False
            
            visted.add(currNode)

            for neighbour, edge_cost in self.graph[currNode]:
                if neighbour in visted:
                    continue

                path.append(currNode)

                if DLS(neighbour, currDepth - 1, path, visted):
                    return True
                
                path.pop()

        for currDepth in range(maxDepth):
            visted, path = set(), []
            if DLS(start, currDepth, path, visted):
                return path
            
        return "unreachable"
    
    def astar(self, start, goal):

        queue = PriorityQueue()
        visted = set()

        heuristic_cost = self.heuristic(start, goal)
        queue.put((0 + heuristic_cost, start))

        total_cost = {start: 0}
        path = {start: None}

        while not queue.empty():
            curr_cost, node = queue.get()

            if node == goal:
                answerPath = []
                while node:
                    answerPath.append(node)
                    node = path[node]

                answerPath.reverse()
                return (" => ".join(answerPath), "   total cost:", total_cost[goal])
            
            visted.add(node)

            for neighbour, cost in self.graph[node]:
                if neighbour in visted:
                    continue

                curr_total_cost = curr_cost + cost

                # not evaluated yet but put in queue
                pending = [node for cost, node in queue.queue]


                if neighbour not in pending:
                    queue.put((curr_total_cost + self.heuristic(node, goal), neighbour))

                elif curr_total_cost >= total_cost[neighbour]:
                    continue

                path[neighbour] = node
                total_cost[neighbour] = curr_total_cost

        return " Unreachable"




        
    def heuristic(self, start, goal):
        return sqrt((self.location[start][0] - self.location[goal][0]) ** 2 + (self.location[start][1] - self.location[goal][1]) ** 2)
        

    def bidirectionalSearch(self, start, goal):
        
        return
    


            





if __name__ == "__main__":
    graph = Graph()
# graph.addNode("Arad", [("Timisoara", 75), ("Zerind", 85), ("Sibiu", 90)])
  
    # graph.addNeighbour("Timisoara", "Arad", 75)
    # print(graph.printGraph())
    # graph.addNeighbours("Timisoara", [("Adama",60), ("Bure", 50)])
    # graph.addNode("Bahirdar")
    # graph.addNeighbours("Bahirdar", [("addis", 60)])
    # print(graph.graph)
    # print(graph.UCS("Arad","Bure"))
    # print(graph.iterativeDeepening("Arad", "Bure", 5))
   
    graph.addNode('Arad', [('Sibiu', 140), ('Timisoara', 118), ('Zerind',75 )])
    graph.addNode('Sibiu', [('Arad', 140), ('Oradea', 151), ('Fagaras', 99), ('Rimnicu Vilcea', 80)])
    graph.addNode('Zerind', [('Oradea', 71), ('Arad',75 )])
    graph.addNode('Timisoara', [('Arad', 118), ('Lugoj', 111)])
    graph.addNode('Lugoj', [('Timisoara', 111), ('Mehadia', 70)])
    graph.addNode('Mehadia', [('Lugoj', 70), ('Drobeta', 75)])
    graph.addNode('Drobeta', [('Mehadia', 75), ('Craiova', 120)])
    graph.addNode('Craiova', [('Drobeta', 120), ('Rimnicu Vilcea', 146), ('Pitesti', 138)])
    graph.addNode('Rimnicu Vilcea', [('Sibiu', 80), ('Craiova', 146), ('Pitesti', 97)])
    graph.addNode('Oradea', [('Sibiu', 151),('Zerind', 71) ])
    graph.addNode('Fagaras', [('Sibiu', 99), ('Bucharest', 211)])
    graph.addNode('Pitesti', [('Rimnicu Vilcea', 97), ('Craiova', 138), ('Bucharest', 101)])
    graph.addNode('Bucharest', [('Fagaras', 211), ('Pitesti', 101), ('Urziceni', 85), ('Giurgiu', 90)])
    graph.addNode('Urziceni', [('Vaslui', 142), ('Hirsova', 98)])
    graph.addNode('Hirsova', [('Eforie', 86)])
    graph.addNode('Vaslui', [('Iasi', 92), ('Urziceni', 142)])
    graph.addNode('Iasi', [('Vaslui', 92), ('Neamt', 87)])

    print(graph.astar('Arad','Bucharest'))
  
   

   
    

