

def add_connection(aj_list: list, node_a: str, node_b: str, weight: int=1):
    if node_a in aj_list:
        aj_list[node_a].add((node_b, weight))
    else:
        aj_list[node_a] = set({(node_b, weight)})

    # if node_b in aj_list:
    #     aj_list[node_b].add((node_a, weight))
    # else:
    #     aj_list[node_b] = set({(node_a, weight)})


aj_list = {}


print(aj_list)

add_connection(aj_list, 'A', 'B')

print(aj_list)

add_connection(aj_list, 'A', 'C', 10)

print(aj_list)

add_connection(aj_list, 'C', 'E', 18)

print(aj_list)