#!/usr/bin/python

import rospy
from explorer import ExplorerClient

def main():
    rospy.init_node("explorer_client")

    ec = ExplorerClient()
    ec.server.setup()
    ec.setup()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ec.loop()
        rate.sleep()

if __name__ == "__main__":
    main()
