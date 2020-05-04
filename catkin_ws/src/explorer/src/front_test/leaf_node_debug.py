import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from collections import deque

# Load and setup our map for plotting
map_data = np.loadtxt("data/map.txt")
leaf_node_data = np.loadtxt("data/leaf_nodes.txt")
occupied = []
free = []
unexplored = []
leaf = []
rows, cols = map_data.shape

# Generate a map for viewing in matplotlib by collating cell types
for i in range(250, 425):
    for j in range(300, 425):
        # generate map arrays
        if map_data[i, j] == -1:
            unexplored.append([i, j])
        elif map_data[i, j] < 50:
            free.append([i, j])
        else:
            occupied.append([i, j])
        # generate leaf node array
        if leaf_node_data[i, j] == 1:
            leaf.append([i, j])

occupied = np.array(occupied)
free = np.array(free)
unexplored = np.array(unexplored)
leaf = np.array(leaf)

plt.figure(1)

# Plot the map with Matplotlib
# plt.scatter(unexplored[:, 1], unexplored[:, 0], s=1, marker='x')
plt.scatter(free[:, 1], free[:, 0], s=2, marker='o')
plt.scatter(occupied[:, 1], occupied[:, 0], s=2)
plt.scatter(leaf[:, 1], leaf[:, 0], s=2)

plt.grid()
plt.savefig('leaf_node_viz')