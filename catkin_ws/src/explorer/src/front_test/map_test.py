import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from collections import deque
import argparse as ap

parser = ap.ArgumentParser()
parser.add_argument('map_file', type=str)
parser.add_argument('--plot_frontiers', action='store_true')
parser.add_argument('--plot_nearest', action='store_true')
args = parser.parse_args()

def get_cell_type(value):
    """
    Takes in a value and returns the cell type
    0: unexplored
    1: free space
    2: occupied
    """ 
    if value == -1:
        return 0
    elif value < 50:
        return 1
    else:
        return 2

def children4(point):
    """
    returns the four neighbourhood surrounding a point in an occupancy grid
    """
    x, y = point.copy()
    candidates = np.array([[x+1, y],
                            [x-1, y],
                            [x, y+1], 
                            [x, y-1]])
    return candidates

def world2map(point):
    """
        Takes in points in the world frame and transforms it into the coordinates
        of the occupancy grid.
    """
    scale = 0.05
    ret = point.copy()
    # Translate
    ret[0] -= -10 # map_info.origin.position.x
    ret[1] -= -10 # map_info.origin.position.y

    # Scale
    ret = np.array(ret, dtype=np.float64)
    ret *= (1.0/float(scale))
    # print("Scale: {}\nOG {}\nTransformed {}".format(scale, og, point))

    return ret

def filter_frontiers(frontiers, data):
    """
    Takes in a list of frontiers and gets rid of the ones that have small unexplored areas, 
    they are filtered accoring to self.min_area
    Inputs:
        frontiers: a list where each entry is of the Frontier class
    Output:
        frontier: a list of frontiers where each of the self.big_enough flag has been set if big enough
    """


    # for i in range(len(frontiers)):
    unexplored_cells = 0
    checked_flags = np.zeros_like(data, dtype=bool)
    bfs = deque()
    frontiers = np.array([world2map(p) for p in frontiers])
    centroid = np.array([np.sum(frontiers[:, 0]), np.sum(frontiers[:, 1])], dtype=np.int32) / len(frontiers)
    # start = np.array(world2map(centroid), dtype=int)
    start = np.array([int(centroid[1]), int(centroid[0])])
    print("STARTING: {}".format(start))
    if get_cell_type(data[start[0], start[1]]):
        start = nearest_cell(start, data.shape[0], data.shape[1], data, value=0)
    # print("Start: {}\nCell Value: {} \nInverted {}".format(start, get_cell_type(map[start[0], start[1]]), get_cell_type(map[start[1], start[0]])))
    # start = nearest_unexplored_cell(start, map.shape[0], map.shape[1], map)
    bfs.append(start)
    points = []
    while (bfs and (unexplored_cells < 100)):
        idx = bfs.popleft()
        for p in children4(idx):
            # Check that the cell is free and we haven't visited it before
            cell_type = get_cell_type(data[p[0], p[1]])
            if (cell_type == 0) and not (checked_flags[p[0], p[1]]):
                checked_flags[p[0], p[1]] = True
                bfs.append(p)
                points.append(p)
                unexplored_cells += 1
        # if unexplored_cells >= 50:
        #     frontiers[i].big_enough = 1
    print("Cell Count {}".format(unexplored_cells))
    return np.array(points)

def nearest_cell(start, width, height, data, value=1):
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
            cell_type = get_cell_type(data[p[0], p[1]])
            # if the cell is free then use it
            if cell_type == value:
                return p
            # Check that the cell is not free and we haven't visited it before
            elif (cell_type != value) and not (explored_flags[p[0], p[1]]):
                explored_flags[p[0], p[1]] = True
                bfs.append(p)
    return "FUCK I COULDN'T FIND SHIT"

# Load and setup our map for plotting
data = np.loadtxt(args.map_file)
occupied = []
free = []
unexplored = []
rows, cols = data.shape

# Generate a map for viewing in matplotlib by collating cell types
for i in range(rows):
    for j in range(cols):
        if data[i, j] == -1:
            unexplored.append([i, j])

        elif data[i, j] < 50:
            free.append([i, j])
        else:
            occupied.append([i, j])

occupied = np.array(occupied)
free = np.array(free)
unexplored = np.array(unexplored)

search_initial = np.array([450, 350])
nearest_cell = nearest_cell(search_initial, data.shape[0], data.shape[1], data)

plt.figure(1)

# Plot the map with Matplotlib
plt.scatter(unexplored[:, 1], unexplored[:, 0], s=1, marker='x')
plt.scatter(free[:, 1], free[:, 0], s=2, marker='o')
plt.scatter(occupied[:, 1], occupied[:, 0], s=2)

# Setup different colour for plotting frontiers
x = np.arange(10)
ys = [i+x+(i*x)**2 for i in range(10)]
colors = cm.rainbow(np.linspace(0, 1, len(ys)))

"""
Plotting Frontiers
"""
if args.plot_frontiers:
    fronts = []
    searched_points = []
    for i in range(8):
        front = np.loadtxt("front{}.txt".format(i))
        fronts.append(front)
        searched_points.append(filter_frontiers(front, data))

    fronts = np.array(fronts)
    searched_points = np.array(searched_points)

    for i, points in enumerate(searched_points):
        front = fronts[i]
        front = np.array([world2map(p) for p in front])
        color = colors[i % 10]
        plt.scatter(points[:, 1], points[:, 0], c=color.reshape(1, 4))
        plt.scatter(front[:, 0], front[:, 1], c=color.reshape(1, 4))
        centroid = np.array([np.sum(front[:, 0]), np.sum(front[:, 1])]) / len(front)
        plt.scatter(centroid[0], centroid[1], c='red')

if args.plot_nearest:
    plt.scatter(search_initial[0], search_initial[1], marker='x')
    plt.scatter(nearest_cell[0], nearest_cell[1], marker='x')

plt.grid()
plt.savefig(args.map_file[:-4]+'_{}_{}.png'.format(int(args.plot_frontiers), int(args.plot_nearest)))