#!/usr/bin/python

import rospy
from pyexplorer import ExplorerClient

def main():
    rospy.init_node("explorer_client")

    robot_id = rospy.get_param("~robot_id")

    ec = ExplorerClient(robot_id)
    ec.setup()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        ec.loop()
        rate.sleep()

if __name__ == "__main__":
    main()
