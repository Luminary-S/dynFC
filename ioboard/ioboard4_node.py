#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ioboard class.

python serial port code for 24 io board
the code '55C81900000055' means closing all io port
and the code '55C819FFFFFF55' means opening all io port
You can also control one relay:
first relay open:55C8010155
first relay close:55C8010055
second relay open:55C8020155
second relay close:55C8020055
.
.
.
.
24th relay open:55C8 18 01 55
24th relay close:55C8 18 00 55
@author: Lzyue
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="SerialIo0"
For 8 ioboard
without delaying:
55C8020155 open #port 2
55c8020055 close
"""
import rospy
import time
from std_msgs.msg import String
import serial
from time import sleep

IO_DICT = {
    "11": '55 C8 01 01 55',
    "10": '55 C8 01 00 55',
    "21": '55 C8 02 01 55',
    "20": '55 C8 02 00 55',
    # "32": 
    "88": '55 C7 01 00 55', # inquiry
}

class Io_board():
    def __init__(self, nodename):
        self.nodename = nodename
        self.iostatebuff = []

    def Init_node(self):
        rospy.init_node(self.nodename)

    def getparam(self):
        self.baud = rospy.get_param("baud")
        self.port = rospy.get_param("serial_id")
        self.rate = rospy.get_param("rate")

    def Io_callback(self, msg):
        rospy.loginfo(msg.data)
        cmd = IO_DICT[msg.data]
        self.iostatebuff.append(cmd)
    
    def Io_Sub(self, topicname):
        sub = rospy.Subscriber(topicname+"cmd", String, self.Io_callback)
        pub = rospy.Publisher(topicname, String, queue_size=1)
        return pub

    def Io_read(self, serial):
        data = serial.read(10).decode('utf-8')
        print("receive:",data)
        in_status = data[-6:-4]
        out_status = data[-4:-3]
        out_state_10 = int(state_16,16)
        out_state_2 = '{:08b}'.format(state_10)
        # if out_status == "01":
        #     ret = "81"
        print(data[-6:-4])
        print(data[-4:-3])
        return out_state_2

    def io_pub(self,pub, str):
        pub.Publisher(str)


def main():
    iob = Io_board("Io_board_node")
    iob.Init_node()
    pub = iob.Io_Sub("io_state")
    iob.getparam()
    try:
        serial = serial.Serial(iob.port, iob.baud,
                               timeout=0.5)  # /dev/ttyUSB0
        
        if serial.isOpen():
            rospy.loginfo("open port success")
        else:
            rospy.loginfo("open port failed")
    except:
        print("com error!")

    rate = rospy.Rate(iob.rate)
    # rostopic pub /io_state std_msgs/String "55C8070155"
    while not rospy.is_shutdown():
        #data =recv(serial)
        if len(iob.iostatebuff) != 0:
            iocmd = iob.iostatebuff[-1]
            rospy.loginfo("receive : %s", str(iocmd))
            time.sleep(0.05)
            serial.write(iocmd.decode("hex"))  # 数据写回
            time.sleep(0.05)
            io_status_2 = iob.Io_read(serial)
            iob.io_pub(pub,io_status_2)
        else:
            pass
        # data ='55C81900000055'
        rate.sleep()


if __name__ == '__main__':
    main()