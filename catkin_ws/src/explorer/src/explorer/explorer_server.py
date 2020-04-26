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
        self.sensing_radius = 1 

        self.pub1 = rospy.Publisher('/robot0/target', PoseStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/robot1/target', PoseStamped, queue_size=10)

        self.listener = tf.TransformListener()

    def setup(self):
        if self.initialized:
            # raise ExplorerError("ExplorerServer already initialized!")
            return
        # TODO finalize any setup needed
        while self.map_listener.frontiers == None:
            continue
        self.initialized = True

    def loop(self):
        if not self.initialized:
            # raise ExplorerError("ExplorerServer not initialized!")
            return
        self.test_selection()

    ###############################
    # Helper functions here
    ###############################
    def get_yaw_from_pose(self, robot_pose):
        r_quat = (robot_pose.orientation.x,
                  robot_pose.orientation.y,
                  robot_pose.orientation.z,
                  robot_pose.orientation.w)
        r_theta = tf.transformations.euler_from_quaternion(r_quat)[2] # yaw of robot frame
        return r_theta

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
        angle: front["angle]
        """
        rf_x = frontier_position[0] - robot_position[0] # frontier pos relative to robot
        rf_y = frontier_position[1] - robot_position[1]

        ret = None
        # print("({}, {})".format(rf_x, rf_y))
        if (rf_x > 0 and rf_y > 0):
            ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] + (-angle + 3*np.pi/2))
        elif (rf_x < 0 and rf_y > 0):
            ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] + (angle + np.pi/2))
        elif (rf_x > 0 and rf_y < 0):
            ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] - + (-angle + np.pi/2))
        else:
            ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] - + (angle + np.pi/2))
        return ret

    def test_selection(self):
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot0', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            front = self.select_frontier(self.get_pose_from_tf(trans, rot), self.map_listener.frontiers)
            target_orientation = self.get_target_yaw(trans, euler, front["location"], front["angle"])
            target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
            target_pose = self.get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
            self.pub1.publish(target_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/robot1', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            front = self.select_frontier(self.get_pose_from_tf(trans, rot), self.map_listener.frontiers)
            target_orientation = self.get_target_yaw(trans, euler, front["location"], front["angle"])
            target_orientation = tf.transformations.quaternion_from_euler(target_orientation[0], target_orientation[1], target_orientation[2])
            target_pose = self.get_pose_stamped_from_tf((front["location"][0], front["location"][1], 0), target_orientation)
            self.pub2.publish(target_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    def rot_z(self, angle):
        """
        Return rotation matrix about z-axis given an angle in radians
        """
        om_hat = np.array([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 0]])
        R = np.eye(3) + om_hat*np.sin(angle) + om_hat.dot(om_hat) * (1 - np.cos(angle))

        return R

    def get_homog(self, pos, angle):
        """
            Returns 4x4 homogenous tranform given a rotation angle about z and the position of
            one frame relative to another
        """
        g = np.eye(4)
        g[0:3, 0:3] = self.rot_z(angle)
        g[0:3, 3] = np.array([pos[0], pos[1], 0])
        return g

    def select_frontier(self, robot_pose, frontier_list):
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

        within = []
        outside = []

        #1. Get information about the robots position
        r_x = robot_pose.position.x
        r_y = robot_pose.position.y

        #2. Go through all frontiers and calc the Euclidean distance between them and the robot as well as the angle
        for frontier in frontier_list:
            frontier_store = {}
            x_t = r_x - frontier.centroid[0] # x transformed to robots coordinate frame
            y_t = r_y - frontier.centroid[1] # y transformed to robots coordinate frame
            dist = np.sqrt(x_t**2 + y_t**2)
            angle = np.arctan2(x_t, y_t)
            frontier_store["dist"] = dist
            frontier_store["angle"] = angle
            frontier_store["location"] = frontier.centroid
            #3. Store them in 'within sensing radius' and 'out of sensing radius' data structures
            if dist <= self.sensing_radius:
                within.append(frontier_store)
            else:
                outside.append(frontier_store)

        #TODO: 4. If the 'within' frontiers is not empty then pick the one with the smallest theta (start at 0 to the left of the robot)
        target = None
        best_angle = np.pi - 0.001
        if len(within) > 0:
            for front in within:
                if front["angle"] < best_angle:
                    target = front
                    best_angle = front["angle"]
            return target

        #TODO: 5. If there are no frontiers near the robot then repeat (3) for the frontiers outside the sensing radius
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



