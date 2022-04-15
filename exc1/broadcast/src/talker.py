#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('jonek', Int16, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    frequency = 20
    rate = rospy.Rate(frequency)
    k_int = 1
    n = 4
    while not rospy.is_shutdown():
        #rospy.loginfo(k_int)
        pub.publish(k_int)
        rate.sleep()
        k_int += n

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
