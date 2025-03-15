#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher_node():
    rospy.init_node('simple_publisher', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    print("Publisher node started")
    
    while not rospy.is_shutdown():
        message = "Hello ROS world!"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass