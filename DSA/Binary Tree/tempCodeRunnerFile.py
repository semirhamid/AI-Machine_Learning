class Node:
    def __init__(self,val) -> None:
        self.val = val
        self.left = None
        self.right = None

a = Node("a")
b = Node("b")
c = Node("c")
d = Node("d")
e = Node("e")
f = Node("f")
a.left = b
a.right = c
b.left = d
b.right = e
c.right = f

def traverse(head):
    if not head :
        return
    left = head.left
    print(left.val)
    print(head.val)
    right = head.right
    print(right.val)
    print(head.val)

traverse(a)
