#!/usr/bin/python
#
# map_listener.py
#
# Provides a class to encapsulate accessing and updating the latest map data.
# 

import rospy, enum

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from collections import deque
from visualization_msgs.msg import Marker, MarkerArray
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
                    output.points = np.vstack((output.points, wp))
                    output.size += 1
                    output.centroid += wp

                    bfs_frontier.append(p)
        
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
        self.points = np.array(self.initial)


class MapListener():
    """
    MapListener

    Subscribes to a map topic and stores the most up to date map information.

    Provides useful utility functions for processing the map
    """
    def __init__(self):
        self.initialized = False
        # TODO use numpy representation for occupancy grid
        self.occupancy_grid = None
        self.metadata = None
        self.new_map = False # indicates no new map present

        self.initial_x = rospy.get_param('/robot1/init_pose_x')
        self.initial_y = rospy.get_param('/robot1/init_pose_y')

        # Is it enough to remap these topics at a node level?
        # My guess is yes since there should only need to be a
        # single map listener for the explorer server
        rospy.init_node('map_listener', anonymous=False)
        rospy.Subscriber("/map", OccupancyGrid, self.occupancy_callback)
        # rospy.Subscriber("map_metadata", MapMetaData, self.metadata_callback)

        self.pub = rospy.Publisher("frontier_markers", MarkerArray, queue_size=10)

        rospy.spin()

    def is_initialized(self):
        """
        Indicates that the MapListener has valid data
        """
        return self.initialized

    def setup(self):
        """
        Intended to be called by the map server itself, DO NOT call from explorer.
        Sets up the MapListener to be ready to operate.
        """
        if self.initialized:
            return
        # TODO additional setup here based on map data

        initial = np.array([self.initial_x, self.initial_y])
        self.graph = Graph(initial, self.occupancy_grid)

        if self.occupancy_grid is not None:
            self.initialized = True
        
        return 

    ############################
    # Utility Functions here
    ############################

    def get_frontiers(self, frontiers):
        """
        Returns numpy array of frontiers in map
        """
        output = []
        frontier_point_arrays = []
        for i, front in enumerate(frontiers):
            marker_pose = Pose()
            marker_pose.position.x = front.centroid[0]
            marker_pose.position.y = front.centroid[1]
            marker_pose.orientation.x = 0
            marker_pose.orientation.y = 0
            marker_pose.orientation.z = 0
            marker_pose.orientation.w = 1
            size = np.clip(float(front.size), 0.0, 50.0)
            size /= 100
            
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                          frame_id="map"),
                                          id=i,
                                          type=2,
                                          pose=marker_pose,
                                          scale=Vector3(x=size,y=size,z=size),
                                          color=ColorRGBA(r=1,a=1))
            output.append(marker)
        self.pub.publish(MarkerArray(markers=output))


    ############################
    # Callback functions here
    ############################

    def occupancy_callback(self, msg):
        self.occupancy_grid = msg
        if self.initialized:
            self.graph.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            np.savetxt('/home/toby/front_test/map.txt', self.graph.map)
            frontiers = self.graph.search()
            self.get_frontiers(frontiers)
        self.setup()

if __name__ == "__main__":
    map_listener = MapListener()
        