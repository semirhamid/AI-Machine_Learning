import queue
graph = {
    0 : [8 , 1, 5],
    1 : [0],
    5 : [0 , 8],
    8 : [0 , 5],
    2 : [3 , 4],
    3 : [2 , 4],
    4 : [3 , 2]
}
count = 0


visited = set()

def visit(start):
    q = queue.Queue()
    q.put(start)
    visited.add(start)
    flag = False
    if start not in visited:
        while not q.empty():
            curr = q.get()
            for i in graph[curr]:
                if i not in visited:
                    flag = True
                q.put(i)
                visited.add(i)
            print("running")
    return flag

for i in graph:
    print(visit(i))
print(count)