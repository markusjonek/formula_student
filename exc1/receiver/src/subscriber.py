#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float32

def callback(data):
    q = 0.15
    heard = data.data
    new_number  = heard / q
    second_pub = rospy.Publisher("kthfs/result", Float32, queue_size=1)
    second_pub.publish(new_number)
    #rospy.loginfo(new_number)


def listner():
    rospy.init_node("listner", anonymous=True)
    rospy.Subscriber("jonek", Int16, callback)
    rospy.spin()

if __name__ == "__main__":
    listner()


