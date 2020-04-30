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
from utils import *

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

        while not self.map_listener.is_initialized():
            rospy.sleep(0.5)

        print("[DEBUG] Explorer Server's MapListener is initialized")

        # TODO: NEED TO GET ROBOT SENSING RADIUS
        self.sensing_radius = 4.09000015258789

        self.pub1 = rospy.Publisher('/robot0/target', PoseStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/robot1/target', PoseStamped, queue_size=10)

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
        #self.test_selection()
        return

    ###############################
    # Helper functions here
    ###############################

    def test_selection(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot0', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            front = self.select_frontier(get_pose_from_tf(trans, rot))
            target_orientation = get_target_yaw(trans, euler, front["location"], front["angle"])
            target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
            target_pose = get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
            self.pub1.publish(target_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot1', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            front = self.select_frontier(get_pose_from_tf(trans, rot))
            target_orientation = get_target_yaw(trans, euler, front["location"], front["angle"])
            target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
            target_pose = get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
            self.pub2.publish(target_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    def get_goal_pose(self, trans, rot):
        """
        Takes in the robots pose and returns the next goal pose
        robot_pose: 
                Type: geometry_msgs.msg Pose
                What?: The current pose of the robot in world co-ordinates
        robot_id 
                Type: int, eg 0 for robot0
        Returns: geometry_msgs.msg PoseStamped
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
            frontier_list:
                Type: list of type Frontier (in frontier_search.py)
                What?: A list of all current frontiers in the map
        Outputs:
            goal: 
                Type: geometry_msgs.msg Pose
                What?: the location of the frontier where the direction is the direction of the vector
                    connecting the robot to the frontier.
        """
        frontier_list=self.map_listener.frontiers

        within = []
        outside = []

        #1. Get information about the robots position
        r = from_position(robot_pose.position)
        R = tf.transformations.quaternion_matrix(from_quaternion(robot_pose.orientation))
        R[0:3, 3] = r 
        R_inv = inverse_homog(R)
        # print("R:\n {}\nR_inv:\n {}".format(R, R_inv))
        #2. Go through all frontiers and calc the Euclidean distance between them and the robot as well as the angle
        for frontier in frontier_list:
            frontier_store = {}
            front_homog = np.hstack((frontier.centroid, np.array([0, 1])))
            p = R_inv.dot(front_homog)
            dist = np.sqrt(p[0]**2 + p[1]**2)
            angle = np.arctan2(p[0], p[1])
            frontier_store["dist"] = dist
            if angle < 0:
                angle = angle + 2*np.pi
            frontier_store["angle"] = angle
            frontier_store["location"] = frontier.centroid
            #3. Store them in 'within sensing radius' and 'out of sensing radius' data structures
            if dist <= self.sensing_radius and frontier.big_enough:
                # print("Point is Within\nWorld Front: {}\n Local Front: {}\nAngle: {}".format(front_homog, p, angle))
                within.append(frontier_store)
            elif frontier.big_enough:
                # print("Point is Outside\nWorld Front: {}\n Local Front: {}\nAngle: {}".format(front_homog, p, angle))
                outside.append(frontier_store)

        #4. If the 'within' frontiers is not empty then pick the one with the smallest theta (start at 0 to the left of the robot)
        target = None
        best_angle = 2*np.pi
        if len(within) > 0:
            for front in within:
                if front["angle"] < best_angle:
                    target = front
                    best_angle = front["angle"]
            return target

        #5. If there are no frontiers near the robot then repeat (3) for the frontiers outside the sensing radius
        elif len(outside) > 0:
            for front in outside:
                if front["angle"] < best_angle:
                    target = front
                    best_angle = front["angle"]
            return target
        # Code broken or no frontiers :'(
        else:
            return "No Frontier Found"


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
            except Exception as e:
                print("Exception occurred: {}".format(e))
                response.status_code = ExplorerTargetServiceResponse.FAILURE

        return response
