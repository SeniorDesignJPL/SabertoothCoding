#!/usr/bin/env python3
import rospy
import getch
from std_msgs.msg import Int32MultiArray

def publisher():
    pub = rospy.Publisher('arduino/motors', Int32MultiArray, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(0.25) # 1hz
    while not rospy.is_shutdown():
        # k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        # rospy.loginfo(str(k))
        # if (k==119):
        #     pub.publish('forward')
        msg = Int32MultiArray()
        msg.data = [1, 2000]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass