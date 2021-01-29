#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys,os,time
import math
import serial #导入模块
from dualEncoder import DualEncoder
import rospy,yaml
from std_msgs.msg import String, Float32
# import threading
# import threading
# STRGLO = "" #读取的数据
# BOOL = True  #读取标志位
BAUD_ = 38400
# RESOLUTION =  65536
# Encoder_CMD = {
#     1: "01 03 00 01 00 01 D5 CA",
#     2: "02 03 00 01 00 01 D5 F9"
# }

# class DualEncoder(SerialPort):

#     def __init__(self):
#         super(PortFinder,self).__init__()
#         self.read_buf= []
#         self.tmp_angle = 0


def node_run():
    PORT_SAVE_FILE = "/home/sgl/catkin_new/src/dynforcecontrol/config/port.yaml"
    port_com = ""
    with open(PORT_SAVE_FILE) as f:
        port_dict = yaml.load(f)
        port_com = port_dict["DualEncoder"]

    pub1 = rospy.Publisher("/encoder1",Float32, queue_size=1)
    pub2 = rospy.Publisher("/encoder2",Float32, queue_size=1)
    rospy.init_node("DualEncoder", log_level=rospy.INFO)
    rospy.loginfo("start DualEncoder node...")

    DE = DualEncoder()

    ret = DE.open_dualencoder( port_com, BAUD_)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        # t0 = time.clock()
        # print("start timer: ", t0)
        DE.read_data()

        d1, d2 = DE.read_buf[-1]

        pub1.publish(d1)
        pub2.publish(d2)
        # t1 = time.clock()
        # print("end timer: ",t1)
        # print("total: ", t1 - t0)

        # rate.sleep() 


if __name__ == "__main__":
    node_run()    
   