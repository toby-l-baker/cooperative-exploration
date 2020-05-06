#!/usr/bin/env python  
#
# explorer_server.py
#
# Implements the explorer server class
#

import traceback
import map_listener
import rospy
from enum import Enum
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Pose
from explorer.srv import ExplorerTargetService, ExplorerTargetServiceRequest, ExplorerTargetServiceResponse
from utils import get_pose_stamped_from_tf, get_pose_from_tf, get_target_yaw, inverse_homog, from_quaternion, from_position, homog_from_pose

class Case(Enum):
    NORMAL = 1 # can act as if we are one robot
    IN_PROXIMITY = 2 # another robot is nearby - select goal to spread out
    NO_NEARBY_GOAL = 3 # no goal nearby - select goal to spread us out from other robots

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
        # self.test_selection()
        self.initialized = True

    def loop(self):
        if not self.initialized:
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
                robot_id = 'robot{}'.format(i)
                (trans,rot) = self.listener.lookupTransform('/map', '/'+robot_id, rospy.Time(0))
                self.robot_info[robot_id] = RobotRecord()
                self.robot_info[robot_id].pose = get_pose_stamped_from_tf(trans, rot)
                self.robot_info[robot_id].robot_id = robot_id
                euler = tf.transformations.euler_from_quaternion(rot)
                case = self.check_case(trans, 'robot{}'.format(i))
                # Get the frontier according to which situation we are in 
                if case != Case.NORMAL:
                    front = self.select_frontier_spread(get_pose_from_tf(trans, rot), robot_id)
                else:
                    front = self.select_frontier(get_pose_from_tf(trans, rot))
                if front == None:
                    print("[DEBUG] Test Selection got front == None")
                    return
                target_orientation = get_target_yaw(trans, euler, front["location"], front["angle"])
                target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
                target_pose = get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
                self.robot_info[robot_id].goal = target_pose
                self.debug_pubs[i].publish(target_pose)

            except Exception as e:
                print("[DEBUG] Test Selection encountered an Exception: {}".format(e))
                traceback.print_exc()
    
    def check_case(self, trans, robot_id):
        """
        Description: Takes in the robots location and determines if it is in any of the edge cases, the function returns which situation the robot is in

        Inputs:
            trans: (x, y, z) - equivalent to what lookupTransform returns
            robot_id: the id of the robot we are trying to set a goal for e.g. "robot0"
        Outputs:
            case: either NORMAL, IN_PROXIMITY or NO_NEARBY_GOAL
        """

        pos = self.robot_info[robot_id].pose.pose.position
        frontiers = self.map_listener.frontiers
        
        # 1. Check if we are close to other robots 
        for id_, info in self.robot_info.items():
            # make sure we aren't checking against ourselves
            if id_ == robot_id:
                continue
            other_pos = info.pose.pose.position
            dist = np.sqrt((pos.x - other_pos.x)**2 + (pos.y - other_pos.y)**2)
            if (dist < self.sensing_radius) and (info.goal is not None):
                return Case.IN_PROXIMITY
        
        # 2. Check if there are any frontiers in our sensing radius
        for frontier in frontiers:
            x, y = frontier.centroid
            dist = np.sqrt((pos.x - x)**2 + (pos.y - y)**2)
            if (dist < self.sensing_radius) and frontier.big_enough:
                return Case.NORMAL
            

        # 3. if we have made it this far then all the frontier are far away
        return Case.NO_NEARBY_GOAL

    def get_goal_pose(self, trans, rot, robot_id):
        """
        Description: uses select frontier to find the x, y location off the frontier to go to and constructs a PoseStamped
                    message to be sent to move_base
        Inputs:
            trans: a three tuple returned from tf listener of the robots x, y, z coordinates
            rot: a quad tuple containing the quaternion of the robots pose
            robot_id: id of the robot who is requesting a new goal e.g. "robot0"

        Returns: geometry_msgs.msg PoseStamped of where the robot should go
        """

        euler = tf.transformations.euler_from_quaternion(rot)

        # Check for  edge cases
        case = self.check_case(trans, robot_id)
        print("[DEBUG] {}".format(case))

        # Get the frontier according to which situation we are in 
        if case != Case.NORMAL:
            front = self.select_frontier_spread(get_pose_from_tf(trans, rot), robot_id)
        else:
            front = self.select_frontier(get_pose_from_tf(trans, rot))

        # front = self.select_frontier(get_pose_from_tf(trans, rot))
        if front == None:
            print("Why are we letting the frontier be -1 here?")
            result = PoseStamped()
            result.header.frame_id = "map_merge"
            result.pose.orientation.w = 1
            return result

        # Set target orientation to be direction connecting the robot and the frontier
        target_orientation = get_target_yaw(trans, euler, front["location"], front["angle"])
        target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])

        # Construct Pose Message
        target_pose = get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)

        return target_pose
    
    def select_frontier_spread(self, robot_pose, robot_id):
        """
        Function to be called if a robot is within sensing distance of another, it checks how far the robot will be from the others goals when
        it reaches its goal. It then selects the frontier that maximizes this distance. Works for n robots and m frontiers. Complexity O(nm)

        Inputs:
            robot_pose: 
                Type: geometry_msgs.msg Pose
                What?: The current pose of the robot in world co-ordinates
        Outputs:
            target: 
                Type: a dictionary containing frontier information
                Keys[angle", "location"]: 
                    ccw angle from robot y axis to point, 
                    x, y coords of the point,
        """
        
        frontier_list = self.map_listener.frontiers

        # 1. Iterate over all frontiers and calculate distance between it and the other robots goals
        # 2. While we are iterating keep checking which is the best (the one with the max resultant separation)
        target = None
        best_distance = 0
        for frontier in frontier_list:
            x, y = frontier.centroid
            dist = 0
            # check the distance from this frontier to the other robots goals
            for id_, info in self.robot_info.items():
                # make sure we aren't checking against ourselves
                if id_ == robot_id:
                    continue
                other_goal = info.goal.pose.position
                dist += np.sqrt((x - other_goal.x)**2 + (y - other_goal.y)**2) # dist rfom frontier to other robots goal
            # if this is the best frontier we have found then store it (must also be big enough and not blacklisted)
            if dist > best_distance and frontier.big_enough and not frontier.blacklisted:
                best_distance = dist
                target = {}
                g = homog_from_pose(robot_pose)
                g_inv = inverse_homog(g)
                p = g_inv.dot(np.hstack((frontier.centroid, np.array([0, 1])))) # get frontier in robot coordinate frame
                angle = np.arctan2(p[0], p[1])
                if angle < 0: # shift from [-pi, pi] to [0, 2pi]
                    angle = angle + 2*np.pi
                target["angle"] = angle # store parameters
                target["location"] = frontier.centroid

        return target

    def select_frontier(self, robot_pose):
        """
        Function to be called when the robot has a nearby goal and is not near other robots it finds the location of each frontier in the robots coordinate 
        frame and then picks the best frontier according to the method outlined in the energy efficient exploration paper

        Inputs:
            robot_pose: 
                Type: geometry_msgs.msg Pose
                What?: The current pose of the robot in world co-ordinates
        Outputs:
            target: 
                Type: a dictionary containing frontier information
                Keys["dist", "angle", "location"]: 
                    distance from the robot to the frontier, 
                    ccw angle from robot y axis to point, 
                    x, y coords of the point,
        """
        frontier_list=self.map_listener.frontiers
        # for deciding which frontier to use
        best_angle = 2*np.pi 
        target = None

        #1. Get information about the robots position
        g = homog_from_pose(robot_pose)
        g_inv = inverse_homog(g)


        #2. Go through all frontiers and calc the Euclidean distance between them and the robot as well as the angle
        for frontier in frontier_list:
            frontier_store = {}
            front_homog = np.hstack((frontier.centroid, np.array([0, 1]))) # convert frontier location to a homogeneous representation
            p = g_inv.dot(front_homog) # frontier location in local coordinates
            dist = np.sqrt(p[0]**2 + p[1]**2) 
            angle = np.arctan2(p[0], p[1])
            frontier_store["dist"] = dist
            if angle < 0: # shift from [-pi, pi] to [0, 2pi]
                angle = angle + 2*np.pi
            frontier_store["angle"] = angle # store parameters
            frontier_store["location"] = frontier.centroid
            #3. Store them in 'within sensing radius' and 'out of sensing radius' data structures
            if (dist <= self.sensing_radius) and frontier.big_enough and not frontier.blacklisted:
                #4. pick the frontier with the smallest theta (start at 0 to the left of the robot)
                if (frontier_store["angle"] < best_angle):
                    target = frontier_store
                    best_angle = frontier_store["angle"]
        if target is not None:
            return target
        else:
            return None


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
            response.target_position = previous_goal
        if request_type == ExplorerTargetServiceRequest.GET_TARGET:
            try:
                trans = robot_pose.pose.position
                trans = [trans.x, trans.y, trans.z]
                rot = robot_pose.pose.orientation
                rot = [rot.x, rot.y, rot.z, rot.w]
                response.target_position = self.get_goal_pose(trans, rot, robot_id)
                response.status_code = ExplorerTargetServiceResponse.SUCCESS
                print("[DEBUG] GET_TARGET Request arrived from {}".format(robot_id))
                # Store new goal with robot info
                self.robot_info[robot_id].goal = response.target_position
            except Exception as e:
                print("Exception occurred with GET_TARGET: {}".format(e))
                print("robot_id = {}".format(robot_id))
                traceback.print_exc()
                response.status_code = ExplorerTargetServiceResponse.FAILURE
        if request_type == ExplorerTargetServiceRequest.BLACKLIST:
            try:
                trans = robot_pose.pose.position
                trans = [trans.x, trans.y, trans.z]
                rot = robot_pose.pose.orientation
                rot = [rot.x, rot.y, rot.z, rot.w]
                to_blacklist = [previous_goal.pose.position.x, previous_goal.pose.position.y]
                self.map_listener.add_to_blacklist(to_blacklist)
                response.target_position = self.get_goal_pose(trans, rot, robot_id)
                response.status_code = ExplorerTargetServiceResponse.SUCCESS
                print("[DEBUG] BLACKLIST Request arrived from {}".format(robot_id))
                # Store new goal with robot info
                self.robot_info[robot_id].goal = response.target_position
            except Exception as e:
                print("Exception occurred with BLACKLIST: {}".format(e))
                print("robot_id = {}".format(robot_id))
                traceback.print_exc()
                response.status_code = ExplorerTargetServiceResponse.FAILURE

        if response.target_position.pose.position.x == 0 and response.target_position.pose.position.y == 0 and response.target_position.pose.position.z == 0:
            print("[ALERT] Would send goal of all zeros")

        print("[DEBUG] Handled request with response {}".format(response))
        return response
