#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
IO board control panel test
"""


import rospy
import time
from std_msgs.msg import String
import serial
from time import sleep

def Io_callback(msg):
    print("IO status:",msg)


def main():
    rospy.init_node("Io_control_node")
    # iob.Init_node()
    topicname = "io_state"
    sub = rospy.Subscriber(topicname, String, Io_callback)
    pub = rospy.Publisher(topicname+"cmd", String, queue_size=1)

    rate = rospy.Rate(30.0)
    while not self.is_shutdown():
        try:
            print("===in python 3 it is input, in python2 should use raw_input ====")
            inp = raw_input(
                "open io port choice(q: quit)? [10,11,20,21], [p1 open, p1 close, p2 open, p2 close: ")
            if inp == "q":
                return
            pub.Publisher(inp)
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
        rate.sleep()


if __name__ == '__main__':
    main()