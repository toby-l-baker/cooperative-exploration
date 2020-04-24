import numpy as np
import matplotlib.pyplot as plt
from collections import deque


plt.figure(1)

def get_cell_type(value):
    if value == -1:
        return 0
    elif value < 50:
        return 1
    else:
        return 2

def children4(point):
    x, y = point.copy()
    candidates = np.array([[x+1, y],
                            [x-1, y],
                            [x, y+1], 
                            [x, y-1]])
    return candidates

def nearest_free_cell(start, width, height, map):
    """
        The start point may not necessarily be free so we need to do a BFS to find the nearest point to that which is free
    """
    bfs = deque()
    bfs.append(start)
    explored_flags = np.zeros((width, height), dtype=bool)
    print("Starting Search at {}".format(start))
    while bfs:
        idx = bfs.popleft()
        for p in children4(idx):
            cell_type = get_cell_type(map[p[0], p[1]])
            # if the cell is free then use it
            if cell_type == 1:
                return p
            # Check that the cell is not free and we haven't visited it before
            elif (cell_type != 1) and not (explored_flags[p[0], p[1]]):
                explored_flags[p[0], p[1]] = True
                bfs.append(p)
    return "FUCK I COULDN'T FIND SHITMA"

data = np.loadtxt("map.txt")
occupied = []
free = []
unexplored = []
rows, cols = data.shape

for i in range(rows):
    for j in range(cols):
        if data[i, j] == -1:
            unexplored.append([i, j])

        elif data[i, j] < 50:
            free.append([i, j])
        else:
            occupied.append([i, j])
    print(i)

occupied = np.array(occupied)
free = np.array(free)
unexplored = np.array(unexplored)

pissma = np.array([300, 370])
suckme = nearest_free_cell(pissma, data.shape[0], data.shape[1], data)

plt.scatter(unexplored[:, 1], unexplored[:, 0], marker='x')
plt.scatter(free[:, 1], free[:, 0], marker='o')
plt.scatter(occupied[:, 1], occupied[:, 0])
plt.scatter(pissma[0], pissma[1], marker='x')
plt.scatter(suckme[0], suckme[1], marker='x')
plt.grid()
plt.show()