#!/usr/bin/python
import time
import rospy

from rosgraph_msgs.msg import Clock


rospy.init_node('clock')
pub = rospy.Publisher('/clock', Clock, queue_size=1)
epoch = rospy.Time(1546300800, 0)

sleep_time = 0.05
percent_sim_time = rospy.get_param("/sim_time_rate", 0.5)
delta_duration = rospy.Duration(sleep_time * percent_sim_time)

while not rospy.is_shutdown():
    epoch += delta_duration
    pub.publish(epoch)
    time.sleep(sleep_time)
