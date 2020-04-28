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
from frontier_search import Graph


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

        # variable to store all frontiers
        self.frontiers = None

        # Is it enough to remap these topics at a node level?
        # My guess is yes since there should only need to be a
        # single map listener for the explorer server
        rospy.Subscriber("/map", OccupancyGrid, self.occupancy_callback)

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

        if (self.occupancy_grid is not None) and (self.initial_x is not None) and (self.initial_y is not None):
            initial = np.array([self.initial_x, self.initial_y])
            min_area = rospy.get_param('/min_frontier_area')
            min_size = rospy.get_param('/min_frontier_size')
            self.graph = Graph(initial, self.occupancy_grid, min_area, min_size)
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
            # np.savetxt('/home/toby/Documents/berkeley/robotics/cooperative-exploration/catkin_ws/src/explorer/src/front_test/front{}.txt'.format(i), front.points)
            col = None
            if front.big_enough == 1:
                col = ColorRGBA(r=1,a=1)
            else:
                col = ColorRGBA(b=1,a=1)
            marker = Marker(header=Header(stamp=rospy.Time.now(),
                                          frame_id="map"),
                                          id=i,
                                          type=2,
                                          pose=marker_pose,
                                          scale=Vector3(x=size,y=size,z=size),
                                          color=col)
            output.append(marker)
        self.pub.publish(MarkerArray(markers=output))


    ############################
    # Callback functions here
    ############################

    def occupancy_callback(self, msg):
        self.occupancy_grid = msg
        if self.initialized:
            self.graph.map = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # For saving maps
            # np.savetxt('/home/toby/Documents/berkeley/robotics/cooperative-exploration/catkin_ws/src/explorer/src/front_test/map.txt', self.graph.map)
            self.frontiers = self.graph.search()
            self.publish_frontier_markers(self.frontiers)
        self.setup()

if __name__ == "__main__":
    map_listener = MapListener()
        