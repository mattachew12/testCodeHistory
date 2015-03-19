#!/usr/bin/env python
import rospy
from mocapGUI.msg import FunctionCall

def callback(msg):
    rospy.loginfo(msg.functionName)
    rospy.loginfo(msg.arguments)

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", FunctionCall, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
