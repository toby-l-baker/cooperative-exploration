#!/usr/bin/env python  
#
# explorer_server.py
#
# Implements the explorer server class
#
import map_listener
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Pose
from explorer.srv import ExplorerTargetService, ExplorerTargetServiceRequest, ExplorerTargetServiceResponse
from utils import get_pose_stamped_from_tf, get_pose_from_tf, get_target_yaw, inverse_homog, from_quaternion, from_position

class RobotRecord():
    """
    RobotRecord

    Class to organize data related to a robot

    Attributes:
    robot_id -- the name of the robot - string
    pose -- the last known location of the robot - PoseStamped
    goal -- the robot's current goal - PoseStamped
    """
    def __init__(self, explorer_request=None):
        """
        Initialize the robot record with an optional explorer request message
        """
        self.robot_id = None
        self.pose = None
        self.goal = None
        if explorer_request is not None:
            self.robot_id = explorer_request.robot_id
            self.pose = explorer_request.robot_pose

class ExplorerServer():
    """
    ExplorerServer

    Organizes multi-robot duties for exploring a map.
    Communicates with an ExplorerClient.
    """
    def __init__(self):
        print("[DEBUG] __init__ ExplorerServer")
        self.initialized = False
        # Get ros parameters and construct objects here
        self.map_listener = map_listener.MapListener()
        self.robot_info = {}

        while not self.map_listener.is_initialized():
            rospy.sleep(0.5)

        print("[DEBUG] Explorer Server's MapListener is initialized")

        # TODO: NEED TO GET ROBOT SENSING RADIUS FROM PARAMETER SERVER
        self.sensing_radius = 4.09000015258789

        self.debug_pubs = []
        self.n_robots = 2
        for i in range(self.n_robots):
            self.debug_pubs.append(rospy.Publisher('/robot{}/target'.format(i), PoseStamped, queue_size=10))

        self.listener = tf.TransformListener()
                
        rospy.Service("explorer_target", ExplorerTargetService, self.service_callback)


    def setup(self):
        if self.initialized:
            return
        # TODO finalize any setup needed
        while self.map_listener.frontiers == None:
            rospy.sleep(0.5)
        print("[DEBUG] Explorer Server is initialized")
        self.initialized = True

    def loop(self):
        if not self.initialized:
            return
        # self.test_selection()
        return

    ###############################
    # Helper functions here
    ###############################

    def test_selection(self):
        """
        This function is for debugging the frontier selection process by visualizing where the targets would be for two robots without
        ever setting the goal positions
        """
        for i in range(self.n_robots):
            try:
                (trans,rot) = self.listener.lookupTransform('/map', '/robot{}'.format(i), rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                front = self.select_frontier(get_pose_from_tf(trans, rot))
                if front == -1:
                    return
                target_orientation = get_target_yaw(trans, euler, front["location"], front["angle"])
                target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
                target_pose = get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
                self.debug_pubs[i].publish(target_pose)

            except Exception as e:
                print("Server cannot execute map transform: {}".format(e))
    
    def get_goal_pose(self, trans, rot):
        """
        Inputs:
            trans: a three tuple returned from tf listener of the robots x, y, z coordinates
            rot: a quad tuple containing the quaternion of the robots pose

        Returns: geometry_msgs.msg PoseStamped of where the robot should go
        """
        euler = tf.transformations.euler_from_quaternion(rot)
        # Get the frontier info 
        front = self.select_frontier(get_pose_from_tf(trans, rot))
        # Set target orientation to be direction connecting the robot and the frontier
        target_orientation = get_target_yaw(trans, euler, front["location"], front["angle"])
        target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
        # Construct Pose Message
        target_pose = get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
        return target_pose

    def select_frontier(self, robot_pose):
        """
        Function to be called when the each robot requests a new goal

        Inputs:
            robot_pose: 
                Type: geometry_msgs.msg Pose
                What?: The current pose of the robot in world co-ordinates
        Outputs:
            target: 
                Type: a dictionary containing frontier information
                Keys["dist", "angle", "location", "blacklist"]: 
                    distance from the robot to the frontier, 
                    ccw angle from robot y axis to point, 
                    x, y coords of the point,
                    if the frontier is blacklisted (i.e. dont use it)
        """
        frontier_list=self.map_listener.frontiers

        within = []
        outside = []

        #1. Get information about the robots position
        r = from_position(robot_pose.position) # get the robots position as a numpy array
        R = tf.transformations.quaternion_matrix(from_quaternion(robot_pose.orientation)) # get the rotation matrix representing the robot's base frame (a 4x4 homogeneous representation)
        R[0:3, 3] = r # set p of homogeneous transform
        R_inv = inverse_homog(R)
        # print("R:\n {}\nR_inv:\n {}".format(R, R_inv))
        #2. Go through all frontiers and calc the Euclidean distance between them and the robot as well as the angle
        for frontier in frontier_list:
            frontier_store = {}
            front_homog = np.hstack((frontier.centroid, np.array([0, 1]))) # convert frontier location to a homogeneous representation
            p = R_inv.dot(front_homog) 
            dist = np.sqrt(p[0]**2 + p[1]**2) 
            angle = np.arctan2(p[0], p[1])
            frontier_store["dist"] = dist
            if angle < 0: # shift from [-pi, pi] to [0, 2pi]
                angle = angle + 2*np.pi
            frontier_store["angle"] = angle # store parameters
            frontier_store["location"] = frontier.centroid
            #3. Store them in 'within sensing radius' and 'out of sensing radius' data structures
            if dist <= self.sensing_radius and frontier.big_enough:
                within.append(frontier_store)
            elif frontier.big_enough:
                outside.append(frontier_store)

        #4. If the 'within' frontiers is not empty then pick the one with the smallest theta (start at 0 to the left of the robot)
        target = None
        best_angle = 2*np.pi
        if len(within) > 0:
            for front in within:
                if (front["angle"] < best_angle):
                    target = front
                    best_angle = front["angle"]
            return target

        #5. If there are no frontiers near the robot then repeat (3) for the frontiers outside the sensing radius
        elif len(outside) > 0:
            for front in outside:
                if (front["angle"] < best_angle):
                    target = front
                    best_angle = front["angle"]
            return target
        # Code broken or no frontiers :'(
        else:
            return -1


    ###############################
    # Callback functions here
    ###############################

    def service_callback(self, request):
        # TODO condition off of the request message
        request_type = request.request_type
        robot_id = request.robot_id
        robot_pose = request.robot_pose
        response = ExplorerTargetServiceResponse()
        response.robot_id = robot_id
        previous_goal = request.previous_goal

        # Store robot position with robot info
        if self.robot_info.get(robot_id, None) is None:
            self.robot_info[robot_id] = RobotRecord(request)
        self.robot_info[robot_id].pose = robot_pose

        if request_type == ExplorerTargetServiceRequest.DEBUG:
            print("[DEBUG] Request arrived from {}".format(robot_id))
            response.status_code = ExplorerTargetServiceResponse.SUCCESS
        if request_type == ExplorerTargetServiceRequest.GET_TARGET:
            try:
                trans = robot_pose.pose.position
                trans = [trans.x, trans.y, trans.z]
                rot = robot_pose.pose.orientation
                rot = [rot.x, rot.y, rot.z, rot.w]
                response.target_position = self.get_goal_pose(trans, rot)
                response.status_code = ExplorerTargetServiceResponse.SUCCESS
                print("[DEBUG] GET_TARGET Request arrived from {}".format(robot_id))
            except Exception as e:
                print("Exception occurred: {}".format(e))
                response.status_code = ExplorerTargetServiceResponse.FAILURE
        if request_type == ExplorerTargetServiceRequest.BLACKLIST:
            try:
                trans = robot_pose.pose.position
                trans = [trans.x, trans.y, trans.z]
                rot = robot_pose.pose.orientation
                rot = [rot.x, rot.y, rot.z, rot.w]
                to_blacklist = [previous_goal.pose.position.x, previous_goal.pose.position.y]
                self.map_listener.blacklist.append(to_blacklist)
                response.target_position = self.get_goal_pose(trans, rot)
                response.status_code = ExplorerTargetServiceResponse.SUCCESS
                print("[DEBUG] BLACKLIST Request arrived from {}".format(robot_id))
            except Exception as e:
                print("Exception occurred: {}".format(e))
                response.status_code = ExplorerTargetServiceResponse.FAILURE

        # Store new goal with robot info
        self.robot_info[robot_id].goal = response.target_position

        return response
