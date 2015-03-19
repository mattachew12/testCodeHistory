#!/usr/bin/env python
# license removed for brevity
import rospy
from mocapGUI.msg import FunctionCall

def talker():
    pub = rospy.Publisher('custom_chatter', FunctionCall)
    rospy.init_node('custom_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    msg = FunctionCall()
    msg.functionName = "something"
    msg.arguments = ("1", "2", "data")

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
