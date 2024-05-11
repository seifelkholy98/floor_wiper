#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import random

def dirt_generator():
    rospy.init_node('dirt_sensor', anonymous=True)
    pub = rospy.Publisher('/dirt_location', Point, queue_size=10)
    rate = rospy.Rate(0.1)  

    while not rospy.is_shutdown():
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        dirt_spot = Point(x=x, y=y, z=0)
        rospy.loginfo(f"Generated dirt at position: ({dirt_spot.x}, {dirt_spot.y})")
        pub.publish(dirt_spot)
        rate.sleep()

if __name__ == '__main__':
    try:
        dirt_generator()
    except rospy.ROSInterruptException:
        pass
