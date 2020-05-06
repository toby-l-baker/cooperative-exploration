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
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseGoal
from explorer.srv import ExplorerTargetService, ExplorerTargetServiceRequest
import utils

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult

import numpy as np

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
        self.close_enough = 0.15 # distance in meters to preempt move_base and get new target
        self.too_far = 400.0 # distance in meters to preempt move_base and get new target
        self._time_exceeded = rospy.get_param("~time_exceeded", 20)
        self._message_delay_time = rospy.Duration(rospy.get_param("~message_delay_time", 5))
        self.MAX_STRIKES = 3
        self.current_strikes = 0
        self.last_update = None
        self.last_message_time = rospy.Time.now()
        self.listener = tf.TransformListener() # for getting robot pose in world frame
        # Stores the current goal travelling to
        self.goal = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id="map_merge"),
                                pose=Pose(position=Point(0, 0, 0),
                                          orientation=Quaternion(0, 0, 0, 1)))

        # Stores the current pose of the robot
        self.pose = None
        while self.pose is None:
            try:
                (trans,rot) = self.listener.lookupTransform('/map_merge', '/{}'.format(self.robot_id), rospy.Time(0))
                self.pose = PoseStamped()
                self.pose.header.frame_id = "/map_merge"
                self.pose.position = Pose(trans[0], trans[1], trans[2])
                self.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            except:
                rospy.sleep(1)

        # Used to detect robot being stuck in place due to MoveBase shenanigans
        self._timeout_callback_tracker = 0
        self.TIMEOUT = 100
        self.RESET_TIME = 100

        # setup dictionary to aid client request decisions
        self.status = {}
        self.status["mb_failure"] = False
        self.status["target_reached"] = False
        self.status["have_a_goal"] = False
        self.status["retry_timer"] = -1
        # Setup Movebase
        self.move_base_api = actionlib.SimpleActionClient('/{}/move_base'.format(robot_id), MoveBaseAction)
        print("[DEBUG] Waiting for MoveBase at /{}/move_base".format(robot_id))
        self.move_base_api.wait_for_server()

        print("[DEBUG] Waiting for explorer server at /explorer_target")
        rospy.wait_for_service('/explorer_target')
        rospy.wait_for_service('/{}/move_base/clear_costmaps'.format(robot_id))
        self.service_proxy = rospy.ServiceProxy('/explorer_target', ExplorerTargetService)
        self.clear_costmaps = rospy.ServiceProxy('/{}/move_base/clear_costmaps'.format(robot_id), Empty)

    def setup(self):
        if self.initialized:
            return 
        # TODO finalize any setup needed
        self.request_publish_goal(ExplorerTargetServiceRequest.GET_TARGET)
        self.last_update = rospy.get_time()
        print("Client Initialized")
        self.initialized = True

    def loop(self):
        # TODO execute main functionality here
        if (rospy.get_time() - self.last_update) > self._time_exceeded:
            # This is so we can better now how the robots are spaced when detecting edge cases
            for k, v in self.status.items():
                print("{}: {}".format(k, v))
            self.last_update = rospy.get_time()
            self.request_publish_goal(ExplorerTargetServiceRequest.DEBUG)

        if self.status["retry_timer"] > 0:
            self.status["retry_timer"] -= 1

        if self.status["retry_timer"] == 0:
            self.status["retry_timer"] = -1
            self.status["have_a_goal"] = False
            self.status["mb_failure"] = False
            self.status["target_reached"] = False

        if self.status["have_a_goal"]:
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

    def send_move_base_goal(self, goal):
        """
        Wrapper around simple action client.

        Arguments:
        goal -- The PoseStamped to request move base to go to
        """
        self.current_strikes = 0
        msg = MoveBaseGoal()
        msg.target_pose = goal
        print("[DEBUG] Sent movebase {}".format(msg))
        self.move_base_api.send_goal(msg, done_cb=self.done_cb, active_cb=None, feedback_cb=self.feedback_cb)
        print("[DEBUG] Api call done")

    def convert_pose_to(self, new_frame, pose_stamped):
        """
        Uses the TF Listener to convert the pose to a new frame
        """
        result = None
        while result is None:
            try:
                pose_stamped.header.stamp = rospy.Time.now()
                result = self.listener.transformPose(new_frame, pose_stamped)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("e: {}".format(e))
                continue
        return result

    def request_publish_goal(self, request_type):
        """ 
        Function should be invoked once on setup and also every time we reach our goal move_
        """
        if (request_type != ExplorerTargetServiceRequest.DEBUG and
                rospy.Time.now() - self.last_message_time < self._message_delay_time):
            return

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
            request.previous_goal = self.convert_pose_to("map", self.goal)
            response = self.service_proxy(request)
            if request_type == ExplorerTargetServiceRequest.DEBUG:
                print("[DEBUG] sent DEBUG request {}".format(request))
                return
            print("[DEBUG] sent request {}".format(request))
            print("[DEBUG] got response {}".format(response))
            new_goal = response.target_position.pose
            if new_goal.position.x == 0 and new_goal.position.y == 0 and new_goal.position.z == 0:
                print("[ALERT] Got an all zero goal position")
                self.status["retry_timer"] = self.RESET_TIME
                self.status["target_reached"] = False
                self.status["have_a_goal"] = True
                self.status["mb_failure"] = False
                self.last_message_time = rospy.Time.now()
                return -1
            self.status["target_reached"] = False
            self.status["have_a_goal"] = True
            self.status["mb_failure"] = False
            self.goal = self.convert_pose_to("map_merge", response.target_position)
            self.send_move_base_goal(self.goal)
            self.goal.header.stamp = rospy.Time.now()
            self.last_message_time = rospy.Time.now()
            # when we get a new goal reset all of the status flags - done in move_base_simple/goal callback
            return 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("[DEBUG] Exception occured while requesting new goal: {}".format(e))
            return 0
        
    ###############################
    # Callback functions here
    ###############################

    def done_cb(self, goal_status, msg):
        """
        Done callback for MoveBase.

        Arguments:
        goal_status -- integer representing the final state of the actionlib. Corresponds to GoalStatus
        msg -- Result message (empty for move_base)
        """
        # TODO Handle other status values
        if goal_status == GoalStatus.SUCCEEDED:
            print("[DEBUG] MoveBase Done CB: Succeeded")
            # Move base has succeeded, clear current goal
            self.status["target_reached"] = True
            self.status["have_a_goal"] = False
            self.status["mb_failure"] = False
        elif goal_status == GoalStatus.ABORTED:
            print("[DEBUG] MoveBase Done CB: Aborted")
            # move base has failed we need to get a new target
            self.status["target_reached"] = False
            self.status["have_a_goal"] = False
            self.status["mb_failure"] = True
        elif goal_status == GoalStatus.PREEMPTED or goal_status == GoalStatus.PREEMPTING:
            print("[DEBUG] MoveBase Done CB: Preempted")
            self.status["target_reached"] = False
            self.status["have_a_goal"] = False
            self.status["mb_failure"] = True
            # DON't DO THIS - it is infinite! Move base got us close enough to preempt, clear current goal and force a different one
        else:
            print("[DEBUG] MoveBase Done CB: Unkown code {}".format(goal_status))


    def feedback_cb(self, msg):
        """
        Feedback callback for MoveBase.

        Arguments:
        msg -- Feedback message containing the current robot pose from move base
        """
        # We want to track how the robot moves:
        # - If the robot has been in the same place for more than TIMEOUT callbacks, raise an alert
        # - If the robot is close to the goal, trigger state to preempt
        new_pose = msg.base_position

        if utils.dist(new_pose, self.pose) < 0.01:
            self._timeout_callback_tracker += 1
        else:
            self._timeout_callback_tracker = 0

        if self._timeout_callback_tracker > self.TIMEOUT:
            print("[ALERT] Probably want to clear costmaps here")
            self.clear_costmaps()
            self._timeout_callback_tracker = 0
            self.current_strikes += 1

        self.pose = new_pose
        if utils.dist(self.pose, self.goal) < self.close_enough:
            print("[ALERT] Canceling goals due to being close enough")
            self.move_base_api.cancel_goal()

        if utils.dist(self.pose, self.goal) > self.too_far:
            print("[ALERT] Canceling goals due to being too far")
            self.move_base_api.cancel_goal()

        if self.current_strikes >= self.MAX_STRIKES:
            print("[ALERT] Canceling goals due to too many stopped strikes")
            self.move_base_api.cancel_goal()
