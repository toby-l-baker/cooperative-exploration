#!/usr/bin/python

import rospy
from explorer import ExplorerServer

def main():
    rospy.init_node("explorer_server")

    es = ExplorerServer()
    es.setup()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        es.loop()
        rate.sleep()

if __name__ == "__main__":
    main()

