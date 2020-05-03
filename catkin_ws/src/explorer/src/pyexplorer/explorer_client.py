#
# explorer_client.py
#
# Implements the explorer client class
#

# from explorer import ExplorerError

import tf
import rospy
import explorer_server
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionResult
from explorer.srv import ExplorerTargetService, ExplorerTargetServiceRequest

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
        # Initial goal to be identity
        self.goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id="map"),
                                pose=Pose(position=Point(0, 0, 0),
                                          orientation=Quaternion(0, 0, 0, 1)))
        self.listener = tf.TransformListener() # for getting robot pose in world frame
        # setup dictionary to aid client request decisions
        self.status = {}
        self.status["mb_failure"] = 0
        self.status["target_reached"] = 0
        self.status["have_a_goal"] = 0
        # Setup Movebase
        self.move_base = rospy.Publisher("{}/move_base_simple/goal".format(robot_id),
                                          PoseStamped, queue_size=10)
        rospy.Subscriber("{}/move_base/result".format(robot_id), MoveBaseActionResult, self.move_base_result_cb)
        rospy.Subscriber("{}/move_base_simple/goal".format(robot_id), PoseStamped, self.move_base_goal_cb)

        rospy.wait_for_service('/explorer_target')
        self.service_proxy = rospy.ServiceProxy('/explorer_target', ExplorerTargetService) # just for testing

    def setup(self):
        if self.initialized:
            return 
        # TODO finalize any setup needed
        self.request_publish_goal(ExplorerTargetServiceRequest.GET_TARGET)
        self.time_since_goal = rospy.get_time()
        print("Client Initialized")
        self.initialized = True

    def loop(self):
        # now = rospy.get_time()
        # If we haven't had a goal for 10 secs get another one
        # if (now - self.time_since_goal) > self._time_exceeded:
        #     self.goal = None
        # TODO execute main functionality here
        # if self.goal is None:
        #     self.request_publish_goal()
        #     self.time_since_goal = rospy.get_time()
        if self.status["have_a_goal"]:
            # could do DEBUG things here if wanted/needed and use time since goal if needed
            pass
        elif self.status["mb_failure"]:
            print("[DEBUG] move_base failed, performing BLACKLIST request")
            self.request_publish_goal(ExplorerTargetServiceRequest.BLACKLIST)
        elif self.status["target_reached"]:
            print("[DEBUG] move_base succeeded, performing GET_TARGET request")
            self.request_publish_goal(ExplorerTargetServiceRequest.GET_TARGET)
        elif not self.status["have_a_goal"]: # happens if our initial goal request fails
            print("[DEBUG] I do not have a goal, performing GET_TARGET request")
            self.request_publish_goal(ExplorerTargetServiceRequest.GET_TARGET)
        

    ###############################
    # Helper functions here
    ###############################

    def request_publish_goal(self, request_type):
        """ 
        Function should be invoked once on setup and also every time we reach our goal move_
        """
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/{}'.format(self.robot_id), rospy.Time(0))
            # (trans,rot) = self.t.lookupTransform("/map", "{}".format(self.robot_id), rospy.Time(0))
            request = ExplorerTargetServiceRequest()
            request.request_type = request_type
            request.robot_id = self.robot_id
            request.robot_pose = PoseStamped()
            request.robot_pose.header.frame_id = "/map"
            request.robot_pose.pose.position = Point(trans[0], trans[1], trans[2])
            request.robot_pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            request.previous_goal = self.goal # self.goal will always be previous until the server responds
            self.response = self.service_proxy(request)
            self.goal = self.response.target_position
            self.move_base.publish(self.goal)
            # when we get a new goal reset all of the status flags - done in move_base_simple/goal callback
            return 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("[DEBUG] Exception occured while requesting new goal: {}".format(e))
            return 0
        
    ###############################
    # Callback functions here
    ###############################

    def move_base_result_cb(self, msg):
        # TODO Handle other status values
        if msg.status.status == 3:
            # Move base has succeeded, clear current goal
            # self.goal = None
            self.status["target_reached"] = 1
            self.status["have_a_goal"] = 0
            self.status["mb_failure"] = 0
        if msg.status.status == 4:
            # move base has failed we need to get a new target
            # self.goal = None
            self.status["target_reached"] = 0
            self.status["have_a_goal"] = 0
            self.status["mb_failure"] = 1
    
    def move_base_goal_cb(self, msg):
        """
        To make sure that have a goal only gets called once we publish to move_base_simple/goal
        """
        self.status["have_a_goal"] = 1
        self.status["target_reached"] = 0
        self.status["mb_failure"] = 0
        print("[DEBUG] published to move_base_simple/goal successfully")
