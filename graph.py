

# G = (V, E)
# G = AJMatrix
# G = AJList

# data
# neighbours

class Node:

    def __init__(self, value) -> None:
        self.value = value
        self.nbs = set([])

class Edge:

    def __init__(self, left : Node, right: Node, w: int = 1) -> None:
        self.left = left
        self.right = right
        self.weight = w

        self.right.nbs.add(left)
        self.left.nbs.add(right)

def print_friends(n: Node):
    x = 6 + 6
    print(f"{n.value}'s Friends: {x}", [f.value for f in n.nbs])
    
A = Node('A')
B = Node('B')
C = Node('C')
D = Node('D')
E = Node('E')

V = [A, B, C, D, E]

e1 = Edge(A, B, 100)
e2 = Edge(A, C, 4)
e3 = Edge(A, E, 8)
e4 = Edge(C, D, 9)

Edges = [e1, e2, e3, e4]

print_friends(A)
print_friends(B)
print_friends(C)
print_friends(D)
print_friends(E)


class Graph:

    def __init__(self, V, E) -> None:
        self.V = V
        self.E = E

    def add_node(self, value):
        pass

    def add_edge(self, node_a, node_b):
        pass

    def search(self, start_node, goal_node):
        pass

Graph(V, Edges)



