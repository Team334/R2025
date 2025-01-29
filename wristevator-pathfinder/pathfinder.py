# Takes in a set of verticies and then forms a complete graph with them. Edges that go through obstacle space can be
# specified in the csv file and are removed from this complete graph. The user then enters two verticies and this program
# will return the shortest path between those two verticies.

import networkx as nx
import matplotlib.pyplot as plt
import csv
import math

with open('wristevator-script/configuration_space.csv', 'r') as file:
    reader = csv.reader(file)
    
    verticies = {}
    dangerous_edges = []

    for line in reader:
        if len(line) == 2:
            dangerous_edges.append(tuple(line))
        
        elif len(line) == 3:
            verticies[line[0]] = (float(line[1]), float(line[2]))
        
        else:
            continue

G = nx.complete_graph(verticies.keys())

# remove all dangerous edges from configuration space
G.remove_edges_from(dangerous_edges)

# find weights for all edges
for v1, v2 in G.edges:
    v1_coords = verticies[v1]
    v2_coords = verticies[v2]

    weight = math.dist(v1_coords, v2_coords)

    G.edges[v1, v2]['weight'] = weight

# shortest path pathfinding
pair = input("Verticex pair to pathfind, enter as v1,v2: ").split(',')
v1, v2 = tuple(pair)

path = nx.shortest_path(G, source=v1, target=v2)
length = sum([
    data['weight'] for v1, v2, data in G.edges(data=True)
    if v1 in path and v2 in path
])

print("Shortest path: " + str(path) + ", Length: " + str(length))

# draw graph
edge_colors = [
    "green" if v1 in path and v2 in path else "black"
    for v1, v2 in G.edges 
]

pos = nx.spring_layout(G)

nx.draw(G, pos, with_labels=True, edge_color=edge_colors, font_weight='bold')
nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, 'weight'), font_size=8)

plt.show()
