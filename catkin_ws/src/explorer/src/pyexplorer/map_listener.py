# map_listener.py
#
# Provides a class to encapsulate accessing and updating the latest map data.
# 
# 

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from frontier_search import Graph, count_free_cells


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

        # Initial search params 
        self.initial_x = None
        self.initial_y = None
        self.min_frontier_area = None
        self.min_frontier_size = None
        
        # for determining how much we have explored
        self.free_cells = None
        self.stdr_map = None
        self.stdr_map_info = None

        # variable to store all frontiers
        self.frontiers = []

        # for the blacklist
        self.blacklist = []
        self.blacklist_thresh = 0.2

        # for determining how often to print
        self.print_i = 0
        self.print_freq = 10

        # Is it enough to remap these topics at a node level?
        # My guess is yes since there should only need to be a
        # single map listener for the explorer server
        rospy.Subscriber("/map", OccupancyGrid, self.occupancy_callback)
        rospy.Subscriber("/stdr/map", OccupancyGrid, self.stdr_map_callback)

        self.pub = rospy.Publisher("frontier_markers", MarkerArray, queue_size=10)

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
        self.initial_x = rospy.get_param('/robot1/init_pose_x')
        self.initial_y = rospy.get_param('/robot1/init_pose_y')
        if (self.occupancy_grid is not None) and (self.initial_x is not None) and (self.initial_y is not None) and (self.stdr_map is not None) and (self.stdr_map_info is not None):
            initial = np.array([self.initial_x, self.initial_y])
            min_area = rospy.get_param('/min_frontier_area')
            min_size = rospy.get_param('/min_frontier_size')
            self.graph = Graph(initial, self.occupancy_grid, min_area, min_size)
            print("[DEBUG] Doing BFS on STDR Map to count number of free cells")
            self.free_cells = count_free_cells(self.stdr_map, self.stdr_map_info)
            self.conversion = (float(self.graph.info.resolution) / self.stdr_map_info.resolution)**2 # stdr map is higher resolution than the gmapping map
            print("[DEBUG] Map has {} free cells and the conversion ratio is {}".format(self.free_cells, self.conversion))
            self.initialized = True
        
        return 

    ############################
    # Utility Functions here
    ############################

    def publish_frontier_markers(self, frontiers):
        """
        Returns numpy array of frontiers in map
        """
        output = []
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

            # For saving frontier points
            # np.savetxt('/home/tobylbaker/cooperative-exploration/catkin_ws/src/explorer/src/front_test/front{}.txt'.format(i), front.points)
            col = None
            if front.blacklisted:
                col = ColorRGBA(a=1) # black
            elif front.big_enough:
                col = ColorRGBA(r=1,a=1) # red 
            else:
                col = ColorRGBA(b=1,a=1) # blue
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                          frame_id="map"),
                                          id=i,
                                          type=2,
                                          pose=marker_pose,
                                          scale=Vector3(x=size,y=size,z=size),
                                          color=col)
            output.append(marker)
        self.pub.publish(MarkerArray(markers=output))


    def add_to_blacklist(self, pt):
        """
        Adds a point to the blacklist, and removes invalid frontiers from the frontier list.

        Arguments:
        pt -- numpy array [x, y] for point to blacklist
        """
        self.blacklist.append(pt)
        self.frontiers, explored_cells = self.graph.search(self.blacklist, self.blacklist_thresh)
        self.publish_frontier_markers(self.frontiers)

    ############################
    # Callback functions here
    ############################

    def occupancy_callback(self, msg):
        self.occupancy_grid = msg
        if self.initialized:
            # self.print_i += 1
            self.graph.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # For saving maps
            # np.savetxt('/home/tobylbaker/cooperative-exploration/catkin_ws/src/explorer/src/front_test/map.txt', self.graph.map)
            self.frontiers, explored_cells = self.graph.search(self.blacklist, self.blacklist_thresh)
            self.publish_frontier_markers(self.frontiers)
            # if (self.print_i % self.print_freq) == 0:
            print("[DEBUG] {}% of the map explored".format((float(explored_cells)/self.free_cells)* self.conversion * 100.0))
                # self.print_i = 0
        self.setup()
    
    def stdr_map_callback(self, msg):
        """
            On startup this will just save the map and in the setup function we do a BFS to see how many free cells there are
            this only gets called once since /stdr/map is a latched topic
        """
        if self.initialized:
            return
        self.stdr_map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        # np.savetxt('/home/tobylbaker/cooperative-exploration/catkin_ws/src/explorer/src/front_test/stdr_map.txt', self.stdr_map)
        self.stdr_map_info = msg.info
        print("[DEBUG] /stdr/map received by map listener")

if __name__ == "__main__":
    map_listener = MapListener()
        
