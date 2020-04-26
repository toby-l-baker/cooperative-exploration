from collections import deque
import enum
import numpy as np

###################
# UTILITY FUNCTIONS
###################

def world2map(point, map_info):
    """
        Takes in points in the world frame and transforms it into the coordinates
        of the occupancy grid.
    """
    scale = map_info.resolution
    ret = point.copy()
    # Translate
    ret[0] -= -10 # map_info.origin.position.x
    ret[1] -= -10 # map_info.origin.position.y

    # Scale
    ret = np.array(ret, dtype=np.float64)
    ret *= (1.0/float(scale))
    # print("Scale: {}\nOG {}\nTransformed {}".format(scale, og, point))

    return ret


def map2world(point, map_info):
    """
        Takes in poses in occupancy grid coordinates and transforms it into world
        coordinates.
    """

    scale = map_info.resolution
    # point = np.array(point, dtype=np.float64)
    # scale
    ret = np.array(point.copy(), dtype=np.float64)
    ret *= float(scale)

    # translate
    ret[0] += -10 # map_info.origin.position.x
    ret[1] += -10 # map_info.origin.position.y

    return ret

class Graph:
    def __init__(self, point, msg): # map is the full ros message
        # Setup map params
        self.info = msg.info
        print(self.info)
        self.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.explored_flags = np.zeros_like(self.map, dtype=bool) # explored cells stored as booleans
        self.frontier_flags = np.zeros_like(self.map, dtype=bool) # frontier cells stored as booleans
        self.frontiers = [] # list of frontiers

        # Setup BFS params
        self.bfs = deque()
        self.head = point
        self.min_frontier_size = 10
    
    def nearest_free_cell(self, start):
        """
            The start point may not necessarily be free so we need to do a BFS to find the nearest point to that which is free
        """
        bfs = deque()
        bfs.append(start)
        explored_flags = np.zeros_like(self.map, dtype=bool)
        print("Starting Search at for free space at {}".format(start))

        if self.get_cell_type(self.map[start[0], start[1]]) == 1:
            return start

        while bfs:
            idx = bfs.popleft()
            for p in self.children4(idx):
                cell_type = self.get_cell_type(self.map[p[0], p[1]])
                # if the cell is free then use it
                if cell_type == 1:
                    return p
                # Check that the cell is not free and we haven't visited it before
                elif (cell_type != 1) and not (explored_flags[p[0], p[1]]):
                    explored_flags[p[0], p[1]] = True
                    bfs.append(p)
        return "Nope"

    def get_cell_type(self, value):
        if value == -1:
            return 0
        elif value < 50:
            return 1
        else:
            return 2

    def children4(self, point):
        x, y = point.copy()
        candidates = np.array([[x+1, y],
                              [x-1, y],
                              [x, y+1], 
                              [x, y-1]])
        return candidates
    
    def children8(self, point):
        x, y = point.copy()
        candidates = np.array([[x+1, y], 
                               [x-1, y],
                               [x, y-1],
                               [x, y+1], 
                               [x-1, y+1],  
                               [x+1, y+1], 
                               [x-1, y-1],  
                               [x+1, y-1]])
        return candidates

    def search(self):
        self.frontiers = [] # frontier cells continually changing so search everytime
        self.explored_flags = np.zeros_like(self.map, dtype=bool)
        self.frontier_flags = np.zeros_like(self.map, dtype=bool)
        start = np.array(world2map(self.head, self.info), dtype=int)
        start = self.nearest_free_cell(start)
        self.bfs.append(start) # may need to alter this to be the nearest free cell
        self.explored_flags[start[0], start[1]] = True
        print("Starting Search at {}".format(start))
        while self.bfs:
            idx = self.bfs.popleft()
            for p in self.children4(idx):
                # Check that the cell is free and we haven't visited it before
                cell_type = self.get_cell_type(self.map[p[0], p[1]])
                if (cell_type == 1) and not (self.explored_flags[p[0], p[1]]):
                    self.explored_flags[p[0], p[1]] = True
                    self.bfs.append(p)
                elif self.isNewFrontierCell(p):
                    self.frontier_flags[p[0], p[1]] = True
                    new_frontier = self.buildNewFrontier(p)
                    if new_frontier.size > self.min_frontier_size:
                        self.frontiers.append(new_frontier)
                        print("New Frontier Added: {}".format((new_frontier.centroid[0], new_frontier.centroid[1])))
        print("Returning Frontier Locations")
        return self.frontiers
                    
    
    def buildNewFrontier(self, initial_cell):
        # Explore uses indexToCells and mapToWorld transforms could cause this to fail
        bfs_frontier = deque()
        bfs_frontier.append(initial_cell)
        initial_world = map2world(np.array([initial_cell[1], initial_cell[0]]), self.info)
        output = Frontier(initial_world)

        while bfs_frontier: # loop til the queue is empty
            idx = bfs_frontier.popleft()
            for p in self.children8(idx):
                if self.isNewFrontierCell(p):
                    self.frontier_flags[p[0], p[1]] = True
                    wp = map2world(np.array([p[1], p[0]]), self.info)
                    output.points.append(wp)
                    output.size += 1
                    output.centroid += wp

                    bfs_frontier.append(p)
        output.points = np.array(output.points)
        output.centroid /= output.size
        return output

    def isNewFrontierCell(self, point):
        # use self.frontier_flags
        # first check that the cell is unknown and not already marked as a frontier
        if (self.map[point[0], point[1]] != -1) or (self.frontier_flags[point[0], point[1]]):
            return False

        # frontier cells have at least one neighbour that is a free cell
        for p in self.children4(point):
            cell_type = self.get_cell_type(self.map[p[0], p[1]])
            if cell_type == 1: # 0 represents free space, 100 is a wall, -1 is an obstacle
                return True

        return False

class Frontier:
    def __init__(self, point):
        self.initial = point
        self.centroid = np.ones(2)*point
        self.size = 1
        self.points = [self.initial]