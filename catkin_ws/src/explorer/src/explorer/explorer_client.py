#
# explorer_client.py
#
# Implements the explorer client class
#

# from explorer import ExplorerError

import tf
import rospy
import explorer_server
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class ExplorerClient():
    """
    ExplorerClient

    Handles robot duties for exploring a map.
    Communicates with an ExplorerServer.
    """
    def __init__(self, robot_id):
        self.initialized = False
        # Get ros parameters and construct objects here
        self.robot_id = robot_id
        self._time_exceeded = rospy.get_param("~time_exceeded", 10)
        self.time_since_goal = None
        self.goal = None
        self.listener = tf.TransformListener() # for getting robot pose in world frame
        # Setup Movebase
        self.move_base = rospy.Publisher("{}/move_base_simple/goal".format(robot_id),
                                          PoseStamped, queue_size=10)
        rospy.Subscriber("{}/move_base/result".format(robot_id), MoveBaseActionResult, self.move_base_result_cb)

        self.server = explorer_server.ExplorerServer() # just for testing

        
    def setup(self):
        if self.initialized:
            return 
        # TODO finalize any setup needed
        self.request_publish_goal()
        self.time_since_goal = rospy.get_time()
        print("Client Initialized")
        self.initialized = True

    def loop(self):
        now = rospy.get_time()
        # If we haven't had a goal for 10 secs get another one
        if (now - self.time_since_goal) > self._time_exceeded:
            self.goal = None
        # TODO execute main functionality here
        if self.goal is None:
            self.request_publish_goal()
            self.time_since_goal = rospy.get_time()

    ###############################
    # Helper functions here
    ###############################

    def request_publish_goal(self):
        """ 
        Function should be invoked once on setup and also every time we reach our goal move_
        """
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/{}'.format(self.robot_id), rospy.Time(0))
            self.goal = self.server.get_goal_pose(trans, rot)
            self.move_base.publish(self.goal)
            return 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("DAMMIT CANNOT GET GOAL")
            return 0
        
    ###############################
    # Callback functions here
    ###############################

    def move_base_result_cb(self, msg):
        # TODO Handle other status values
        if msg.status.status == 3:
            # Move base has succeeded, clear current goal
            self.goal = None
            

