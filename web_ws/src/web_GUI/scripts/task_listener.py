#!/usr/bin/env python3

# Modules import
import rospy

# Messages import
from navibee_msgs.msg import TaskStatus

def task_listener_callback(msg):
    print(msg)

def run():
    rospy.init_node("task_listener", anonymous=False)
    rospy.Subscriber("/navibee/robottaskstatus", TaskStatus, task_listener_callback)

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Waiting for robot task status message.")
        except Exception as e:
            rospy.loginfo(e)

if __name__ == "__main__":
    run()