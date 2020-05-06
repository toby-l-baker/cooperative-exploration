#!/usr/bin/python
#
# path_length.py
#
# A simple node to measure and publish the current path length for a robot.
#
# Expects to be namespaced under the robot's namespace

import rospy
import numpy as np

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse

class PathLengthNode():
    def __init__(self):
        # Used to determine time delta between cmd_vel_callback calls
        self.last_time = None
        # Stores the current path length of the robot
        self.path_length = None
        # self.robot_id = rospy.get_param("~robot_id")
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.pub = rospy.Publisher('path_length', Float64, queue_size=10)

        rospy.Service('reset_path_length', Empty, self.reset)

    def odom_callback(self, odom):
        now = odom.header.stamp
        # initialisation
        if self.last_time is None: 
            self.last_time = now
            self.path_length = 0.0
        else:
            delta_t = now - self.last_time
            self.last_time = now
            twist = odom.twist.twist
            u = twist.linear.x
            v = twist.angular.z

            if np.isclose(u, 0.0):
                # Rotating in place, 1/pi units per radian
                self.path_length += np.abs(v) * delta_t.to_sec() / np.pi
            else:
                # Translating with tangential velocity u, 1 unit per m
                self.path_length += np.abs(u) * delta_t.to_sec()

    def publish_path_length(self):
        if self.path_length is None:
            raise Exception("Path length is still None!")
        self.pub.publish(self.path_length)

    def reset(self, req):
        """
        Resets the path length node by clearing the last time recorded.
        """
        self.last_time = None
        return EmptyResponse()

def main():
    rospy.init_node('path_length')
    node = PathLengthNode()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            node.publish_path_length()

        except Exception as e:
            pass
        finally:
            rate.sleep()

if __name__=="__main__":
    main()
