#
# explorer_client.py
#
# Implements the explorer client class
#

# from explorer import ExplorerError

import tf
import rospy
from explorer_server import ExplorerServer
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class ExplorerClient():
    """
    ExplorerClient

    Handles robot duties for exploring a map.
    Communicates with an ExplorerServer.
    """
    def __init__(self):
        self.initialized = False
        # TODO Get ros parameters and construct objects here
        self.listener = tf.TransformListener() # for getting robot pose in world frame

        self.server = ExplorerServer() # just for testing

        self.num_robots = rospy.get_param('/num_robots')
        self.pubs_mb = []
        for i in range(self.num_robots):
            goal_topic = 'robot{}/move_base_simple/goal'.format(i)
            result_topic = 'robot{}/move_base/result'.format(i)
            self.pubs_mb.append(rospy.Publisher(goal_topic, PoseStamped, queue_size=10))
            # i is a callback arg passed to the subscriber callback
            rospy.Subscriber(result_topic, MoveBaseActionResult, self.move_base_result_cb, i) 
        
    def setup(self):
        if self.initialized:
            # raise ExplorerError("ExplorerClient already initialized!")
            return 
        # TODO finalize any setup needed
        for i in range(self.num_robots):
            print("Sent goal to robot{}".format(i))
            self.request_publish_goal(i)
        print("Client Initialized")
        self.initialized = True

    def loop(self):
        if not self.initialized:
            # raise ExplorerError("ExplorerClient not initialized!")
            return 
        # TODO execute main functionality here

    ###############################
    # Helper functions here
    ###############################

    def request_publish_goal(self, robot_id):
        """ 
        Function should be invoked once on setup and also every time we reach our goal move_
        """
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot{}'.format(robot_id), rospy.Time(0))
            goal = self.server.get_goal_pose(trans, rot)
            self.pubs_mb[robot_id].publish(goal)
            print("told robot{} to go to\n{}".format(robot_id, (goal.pose.position.x, goal.pose.position.y)))
            return 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("DAMMIT")
            return 0
        
    ###############################
    # Callback functions here
    ###############################

    def move_base_result_cb(self, msg, robot_id):
        if msg.status.status == 3:
            # Move base has succeeded, send a new goal
            self.request_publish_goal(robot_id)
            

