#!/usr/bin/python
#
# map_listener.py
#
# Provides a class to encapsulate accessing and updating the latest map data.
# 

import rospy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from collections import deque
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class Graph:
    def __init__(self, point, msg): # map is the full ros message
        # Setup map params
        self.grid_size = (msg.info.width, msg.info.height)
        self.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.explored_flags = np.zeros_like(self.map, dtype=bool) # explored cells stored as booleans
        self.frontier_flags = np.zeros_like(self.map, dtype=bool) # frontier cells stored as booleans
        self.frontiers = [] # list of frontiers

        # Setup BFS params
        self.bfs = deque()
        self.head = point
        self.min_frontier_size = 5

    def children4(self, point):
        x, y = point
        candidates = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        return candidates
    
    def children8(self, point):
        x, y = point
        candidates = [(x+1, y), (x-1, y), (x-1, y+1), (x, y+1), (x+1, y+1), (x-1, y-1), (x, y-1), (x+1, y-1)]
        return candidates

    def search(self):
        self.frontiers = [] # frontier cells continually changing so search
        self.explored_flags = np.zeros_like(self.map, dtype=bool)
        self.frontier_flags = np.zeros_like(self.map, dtype=bool)
        self.bfs.append(self.head) # may need to alter this to be the nearest free cell
        self.explored_flags[self.head[0], self.head[1]] = True

        while self.bfs:
            idx = self.bfs.popleft()
            for x, y in self.children4(idx):
                # Check that the cell is free and we haven't visited it before
                if (self.map[x, y] <= 50) and not (self.explored_flags[x, y]):
                    self.explored_flags[x, y] = True
                    self.bfs.append((x, y))
                elif self.isNewFrontierCell((x, y)):
                    print("OMG FRONTIER CELLS")
                    self.frontier_flags[x, y] = True
                    new_frontier = self.buildNewFrontier((x, y))
                    if new_frontier.size > self.min_frontier_size:
                        self.frontiers.append(new_frontier)
            return self.frontiers
                    
    
    def buildNewFrontier(self, initial_cell):
        # Explore uses indexToCells and mapToWorld transforms could cause this to fail
        bfs_frontier = deque()
        bfs_frontier.append(initial_cell)

        output = Frontier(initial_cell)

        while bfs_frontier: # loop til the queue is empty
            idx = bfs_frontier.popleft()
            for x, y in self.children8(idx):
                if self.isNewFrontierCell((x, y)):
                    self.frontier_flags[x, y] = True
                    output.points.append((x, y))
                    output.size += 1
                    output.centroid_x += x
                    output.centroid_y += y

                    bfs_frontier.append((x, y))
        
        output.centroid_x /= output.size
        output.centroid_y /= output.size
        return output

    def isNewFrontierCell(self, point):
        # use self.frontier_flags
        # first check that the cell is unknown and not already marked as a frontier
        if (self.map[point[0], point[1]] != -1) or (self.frontier_flags[point[0], point[1]]):
            return False

        # frontier cells have at least one neighbour that is a free cell
        for x, y in self.children4(point):
            print(self.map[x, y])
            if self.map[x, y] <= 50 and self.map[x, y] >=0: # 0 represents free space, 100 is a wall, -1 is an obstacle
                return True

        return False

class Frontier:
    def __init__(self, point):
        self.initial = point
        self.centroid_x = 0
        self.centroid_y = 0
        self.size = 1
        self.points = [self.initial]


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

        self.initial_x = rospy.get_param('/robot0/init_pose_x')
        self.initial_y = rospy.get_param('/robot0/init_pose_y')

        # Is it enough to remap these topics at a node level?
        # My guess is yes since there should only need to be a
        # single map listener for the explorer server
        rospy.init_node('map_listener', anonymous=False)
        rospy.Subscriber("map", OccupancyGrid, self.occupancy_callback)
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

        initial = (self.initial_x, self.initial_y)
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
        for i, front in enumerate(frontiers):
            marker_pose = Pose()
            marker_pose.position.x = front.centroid_x
            marker_pose.position.y = front.centroid_y
            marker_pose.orientation.x = 0
            marker_pose.orientation.y = 0
            marker_pose.orientation.z = 0
            marker_pose.orientation.w = 1
            
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                          frame_id="world"),
                                          id=i,
                                          type=2,
                                          pose=marker_pose,
                                          scale=Vector3(x=front.size,y=front.size,z=front.size),
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
            frontiers = self.graph.search()
            self.get_frontiers(frontiers)
        self.setup()

if __name__ == "__main__":
    map_listener = MapListener()
        