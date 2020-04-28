#!/usr/bin/env python  
#
# explorer_server.py
#
# Implements the explorer server class
#
import sys
sys.path.append("explorer/")
from map_listener import MapListener 
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Pose

class ExplorerServer():
    """
    ExplorerServer

    Organizes multi-robot duties for exploring a map.
    Communicates with an ExplorerClient.
    """
    def __init__(self):
        self.initialized = False
        # TODO Get ros parameters and construct objects here
        self.map_listener = MapListener()
        self.map_listener.initial_x = rospy.get_param('/robot1/init_pose_x')
        self.map_listener.initial_y = rospy.get_param('/robot1/init_pose_y')

        # TODO: NEED TO GET ROBOT SENSING RADIUS
        self.sensing_radius = 4.09000015258789

        self.pub1 = rospy.Publisher('/robot0/target', PoseStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/robot1/target', PoseStamped, queue_size=10)

        self.listener = tf.TransformListener()

    def setup(self):
        if self.initialized:
            # raise ExplorerError("ExplorerServer already initialized!")
            return
        # TODO finalize any setup needed
        while self.map_listener.frontiers == None:
            pass
        self.initialized = True

    def loop(self):
        if not self.initialized:
            # raise ExplorerError("ExplorerServer not initialized!")
            return
        self.test_selection()

    ###############################
    # Helper functions here
    ###############################

    def get_pose_stamped_from_tf(self, trans, rot):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp=rospy.Time.now()
        ps.pose.position.x=trans[0]
        ps.pose.position.y=trans[1]
        ps.pose.position.z=trans[2]
        ps.pose.orientation.x=rot[0]
        ps.pose.orientation.y=rot[1]
        ps.pose.orientation.z=rot[2]
        ps.pose.orientation.w=rot[3]
        return ps

    def get_pose_from_tf(self, trans, rot):
        pose = Pose()
        pose.position.x=trans[0]
        pose.position.y=trans[1]
        pose.position.z=trans[2]
        pose.orientation.x=rot[0]
        pose.orientation.y=rot[1]
        pose.orientation.z=rot[2]
        pose.orientation.w=rot[3]
        return pose

    def get_target_yaw(self, robot_position, robot_orientation, frontier_position, angle):
        """
        Returns the desired yaw angle for the robot to reach its goal
        robot_position: (x, y, z)
        robot_orientation: (roll, pitch, yaw)
        frontier_position: front["location"]
        angle: front["angle"] - angle is relative to the robots position 
        """
        # rf_x = frontier_position[0] - robot_position[0] # frontier pos relative to robot
        # rf_y = frontier_position[1] - robot_position[1]

        ret = None
        # print("({}, {})".format(rf_x, rf_y))
        
        if angle < np.pi/2:
            ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] + (-angle + np.pi/2))
        else:
            ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] - (angle-np.pi/2))
        return ret

    def test_selection(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot0', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            front = self.select_frontier(self.get_pose_from_tf(trans, rot))
            target_orientation = self.get_target_yaw(trans, euler, front["location"], front["angle"])
            target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
            target_pose = self.get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
            self.pub1.publish(target_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot1', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            front = self.select_frontier(self.get_pose_from_tf(trans, rot))
            target_orientation = self.get_target_yaw(trans, euler, front["location"], front["angle"])
            target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
            target_pose = self.get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
            self.pub2.publish(target_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    def from_quaternion(self, msg):
        """
        Returns a numpy array from a ROS quaternion
        """
        return np.array([msg.x, msg.y, msg.z, msg.w])
    
    def from_position(self, msg):
        """
        Returns robot position as a numpy array
        """
        return np.array([msg.x, msg.y, msg.z])

    def inverse_homog(self, mat):
        """
        Returns the invers of a homogeneous transformation matrix
        """
        out = np.eye(4)
        R = mat[0:3, 0:3]
        p = mat[0:3, 3]
        out[0:3, 0:3] = R.T
        out[0:3, 3] = -np.transpose(R[0:3, 0:3]).dot(p)
        assert(np.allclose(np.eye(4), mat.dot(out)))
        return out

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
        front = self.select_frontier(self.get_pose_from_tf(trans, rot))
        # Set target orientation to be direction connecting the robot and the frontier
        target_orientation = self.get_target_yaw(trans, euler, front["location"], front["angle"])
        target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
        # Construct Pose Message
        target_pose = self.get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
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
        r = self.from_position(robot_pose.position)
        R = tf.transformations.quaternion_matrix(self.from_quaternion(robot_pose.orientation))
        R[0:3, 3] = r 
        R_inv = self.inverse_homog(R)
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



