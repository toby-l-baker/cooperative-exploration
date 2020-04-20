#
# map_listener.py
#
# Provides a class to encapsulate accessing and updating the latest map data.
# 

import rospy

from nav_msgs.msg import OccupancyGrid, MapMetaData

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

        self.frontiers = None

        # Is it enough to remap these topics at a node level?
        # My guess is yes since there should only need to be a
        # single map listener for the explorer server
        rospy.Subscriber("map", OccupancyGrid, self.occupancy_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.metadata_callback)

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
        self.compute_frontiers()
        if self.occupancy_grid is not None and self.metadata is not None:
            self.initialized = True

    ############################
    # Utility Functions here
    ############################

    def compute_frontiers(self):
        """
        Updates datastructures to compute numpy array of frontiers in map
        """
        raise NotImplementedError()

    def get_frontiers(self):
        """
        Returns numpy array of frontiers in map
        """
        raise NotImplementedError()

    ############################
    # Callback functions here
    ############################

    def occupancy_callback(self, msg):
        self.occupancy_grid = msg
        self.setup()

    def metadata_callback(self, msg):
        self.metadata = msg
        self.setup()
