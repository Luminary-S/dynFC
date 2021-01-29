#!/usr/bin/python
#coding=utf-8
########################################################################
#
# Copyright (c) 2020, CUHK CURI 
# Author: SGL
#
# All rights reserved.
# force basic class for 
#
#
########################################################################


import rosbag


bagDir = "/home/sgl/bagfile/"
bagfile = "force_test1.bag"
fname = bagDir + bagfile

bag = rosbag.Bag(fname, "r")

topicName = "/robotiq_ft_wrench"

bag_data = bag.read_messages(topicName)
t_org = []
t = []
fx = []
fy = []
fz = []
tx = []
ty = []
tz = []

### read topic
i = 0
for topic, msg, tn in bag_data:
    
    t_temp = tn.to_nsec()
    if i == 0:
        t.append(0)
        # t_org.append(t)
        # continue
    # if i < 100:
        # print(t,msg)
    else:
        
        t.append(round((t_temp - t_org[0])*1e-10,5))

    # print(t.to_nsec())
    # print(msg.wrench)
    t_org.append(t_temp)
    
    fx.append( msg.wrench.force.x )
    fy.append( msg.wrench.force.y )
    fz.append( msg.wrench.force.z )
    tx.append( msg.wrench.torque.x )
    ty.append( msg.wrench.torque.y )
    tz.append( msg.wrench.torque.z )
    i +=1


## display
import matplotlib.pyplot as plt


def draw( x,y, name):
    if name.split(".")[0] == "force":
        labelName = "Force(N)"
    elif name.split(".")[0] == "torque":
        labelName = "Torque(N*m)"
    plt.ylabel(labelName)
    plt.xlabel('time(nsec)')
    # plt.xlim((0,120))
    # plt.ylim((-30,30))
    # plt.title("robotiq measurement")
    # Text(0.5, 0, 'index')
    
    plt.plot(x, y, label=name)
    plt.legend(loc='best')
# print(t[:100])
plt.figure()
plt.suptitle("robotiq measurement")
plt.subplot(231)
draw(t,fx, "force.x")

plt.subplot(232)
draw(t,fy, "force.y")

plt.subplot(233)
draw(t,fz, "force.z")

plt.subplot(234)
draw(t,tx, "torque.x")

plt.subplot(235)
draw(t,ty, "torque.y")

plt.subplot(236)
draw(t,tz, "torque.z")


plt.show()