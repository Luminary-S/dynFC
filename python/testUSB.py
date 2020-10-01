#!/usr/bin/python2
# -*- coding: utf-8 -*-
#author: sgl
#date: 20200805
import time
from functools import wraps
import logging
import matplotlib.pyplot as plt
import numpy as np
import math
import usb
import serial
from serial.tools.list_ports import *
# all_devs = usb.core.find(find_all=True)

# # print(all_devs)
# for d in all_devs:
#     # print(d.idVendor,d.idProduct)
#     if (d.idVendor == 0x0471 ) & (d.idProduct == 0x1200):
#         print(d)
#     if (d.idVendor == 0x1a86 ) & (d.idProduct == 0x7523):
#         print(d)

plist = comports()
for d in plist:
    print(d.vid,d.pid,d.hwid)
    # print(d.device)
    if (d.vid == 0x0471 ) & (d.pid == 0x1200):
        print(d.device)
# print(plist[0].device)
# print(plist[0].vid)
# print(plist[0].pid)