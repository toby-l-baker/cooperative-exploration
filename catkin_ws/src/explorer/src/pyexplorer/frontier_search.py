from collections import deque
import enum
import numpy as np
import rospy

###################
# UTILITY FUNCTIONS
###################

def world2map(point, origin, scale):
    """
        Takes in points in the world frame and transforms it into the coordinates
        of the occupancy grid.
    """
    # scale = map_info.resolution
    ret = point.copy()
    # Translate
    ret[0] -= origin[0] # map_info.origin.position.x
    ret[1] -= origin[1] # map_info.origin.position.y

    # Scale
    ret = np.array(ret, dtype=np.float64)
    ret *= (1.0/float(scale))
    # print("Scale: {}\nOG {}\nTransformed {}".format(scale, og, point))

    return ret


def map2world(point, origin, scale):
    """
        Takes in poses in occupancy grid coordinates and transforms it into world
        coordinates.
    """

    # scale = map_info.resolution
    # point = np.array(point, dtype=np.float64)
    # scale
    ret = np.array(point.copy(), dtype=np.float64)
    ret *= float(scale)

    # translate
    ret[0] += origin[0] # map_info.origin.position.x
    ret[1] += origin[1] # map_info.origin.position.y

    return ret

class Graph:
    def __init__(self, point, msg, min_area, min_size): # map is the full ros message
        # Setup map params
        self.info = msg.info
        self.map_origin = (-10, -10)

        # print(self.info)
        self.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.explored_flags = np.zeros_like(self.map, dtype=bool) # explored cells stored as booleans
        self.frontier_flags = np.zeros_like(self.map, dtype=bool) # frontier cells stored as booleans
        self.frontiers = [] # list of frontiers

        # Setup BFS params
        self.bfs = deque()
        self.head = point
        # Set to rosparam values in explorer server setup function
        self.min_frontier_area = min_area # min number of cells unexplored cells in order to be considered a frontier
        self.min_frontier_size = min_size 
        # for determining if we shoul print debug info
        self.print_i = 0 
    
    def nearest_cell(self, start, value=1):
        """
            The start point may not necessarily be free so we need to do a BFS to find the nearest point to that which is free
        """
        bfs = deque()
        bfs.append(start)
        explored_flags = np.zeros_like(self.map, dtype=bool)
        # print("Starting Search at for free space at {}".format(start))

        if self.get_cell_type(self.map[start[0], start[1]]) == 1:
            return start

        while bfs:
            idx = bfs.popleft()
            for p in self.children4(idx):
                cell_type = self.get_cell_type(self.map[p[0], p[1]])
                # if the cell is free then use it
                if cell_type == value:
                    return p
                # Check that the cell is not free and we haven't visited it before
                elif (cell_type != value) and not (explored_flags[p[0], p[1]]):
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

    def search(self, blacklist, thresh, map_=None, map_info=None, search_input_map=False):
        """ 
        BFS Search on the map to find frontiers, by default searches through self.map
        if search_map is set to True it will search the map input.
        """
        origin = self.map_origin
        resolution = self.info.resolution
        if search_input_map == True:
            self.map = map_
            # print("[DEBUG] Using Input Map")
            # TODO: offset origin or head position when working with stdr/map
            origin = (map_info.origin.position.x, map_info.origin.position.y)
            resolution = map_info.resolution

        explored_cells = 0 # for checking % map explored

        frontiers = [] # frontier cells continually changing so search everytime
        self.frontier_flags = np.zeros_like(self.map, dtype=bool)
        self.explored_flags = np.zeros_like(self.map, dtype=bool)
        
        start = np.array(world2map(self.head, origin, resolution), dtype=int)
        start = self.nearest_cell(start)
        self.bfs.append(start) # may need to alter this to be the nearest free cell
        self.explored_flags[start[0], start[1]] = True
        while self.bfs:
            idx = self.bfs.popleft()
            for p in self.children4(idx):
                # Check that the cell is free and we haven't visited it before
                cell_type = self.get_cell_type(self.map[p[0], p[1]])
                if (cell_type == 1) and not (self.explored_flags[p[0], p[1]]):
                    self.explored_flags[p[0], p[1]] = True
                    self.bfs.append(p)
                    explored_cells += 1
                elif self.isNewFrontierCell(p):
                    self.frontier_flags[p[0], p[1]] = True
                    new_frontier = self.buildNewFrontier(p)
                    if new_frontier.size > self.min_frontier_size:
                        frontiers.append(new_frontier)
        self.frontiers = self.filter_frontiers(frontiers, blacklist, thresh)
        return self.frontiers, explored_cells
                    
    def filter_frontiers(self, frontiers, blacklist, thresh):
        """
        Takes in a list of frontiers and gets rid of the ones that have small unexplored areas, 
        they are filtered accoring to self.min_area
        Inputs:
            frontiers: a list where each entry is of the Frontier class
            blacklist: a list of points that the robot has previously been unable to get to
            thresh: how close the frontier needs to be to be considered blacklisted 
        Output:
            frontier: a list of frontiers where each of the self.big_enough flag has been set if big enough
            and a self.blacklisted flag has been set if the frontier has been blacklisted due to proximity to 
            a blacklisted point
        """
        # variable for printing debug info
        n_fronts = 0
        n_small = 0
        n_blacklisted = 0
        self.print_i += 1

        for i in range(len(frontiers)):

            unexplored_cells = 0
            checked_flags = np.zeros_like(self.map, dtype=bool)
            bfs = deque()
            centroid = np.array([frontiers[i].centroid[1], frontiers[i].centroid[0]])

            # First check if we should blacklist the frontier
            for x, y in blacklist:
                dist = np.sqrt((centroid[0] - x)**2 + (centroid[1] - y)**2)
                if dist < thresh:
                    frontiers[i].blacklisted = True
                    n_blacklisted += 1
                    break

            # No need to do a BFS if the point is blacklisted
            if not frontiers[i].blacklisted:
                start = np.array(world2map(centroid, (-10, -10), self.info.resolution), dtype=int)
                if self.get_cell_type(self.map[start[0], start[1]]) != 0:
                    self.nearest_cell(start, value=0) # find the nearest unexplored cell
                bfs.append(start)
                while (bfs and (unexplored_cells < self.min_frontier_area)):
                    idx = bfs.popleft()
                    for p in self.children4(idx):
                        # Check that the cell is free and we haven't visited it before
                        cell_type = self.get_cell_type(self.map[p[0], p[1]])
                        if (cell_type == 0) and not (checked_flags[p[0], p[1]]):
                            checked_flags[p[0], p[1]] = True
                            bfs.append(p)
                            unexplored_cells += 1
                if unexplored_cells >= self.min_frontier_area:
                    frontiers[i].big_enough = True
                    n_fronts += 1
                else:
                    n_small += 1
        if (self.print_i % 10) == 0:
            print("[DEBUG] {} Target Frontiers, {} Blacklisted Frontiers, {} Interior Frontiers".format(n_fronts, n_blacklisted, n_small))
            self.print_i = 0
        return frontiers

    def buildNewFrontier(self, initial_cell):
        # Explore uses indexToCells and mapToWorld transforms could cause this to fail
        bfs_frontier = deque()
        bfs_frontier.append(initial_cell)
        initial_world = map2world(np.array([initial_cell[1], initial_cell[0]]), self.map_origin, self.info.resolution)
        output = Frontier(initial_world)
        while bfs_frontier: # loop til the queue is empty
            idx = bfs_frontier.popleft()
            for p in self.children8(idx):
                if self.isNewFrontierCell(p):
                    self.frontier_flags[p[0], p[1]] = True
                    wp = map2world(np.array([p[1], p[0]]), self.map_origin, self.info.resolution)
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
        self.big_enough = False
        self.blacklisted = False