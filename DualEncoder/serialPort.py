#!/usr/bin/python3
# -*- coding: utf-8 -*-

import serial
import serial.tools.list_ports
import re
# import argparse
import sys
from binascii import *
import crcmod


''' base class of serial port,  
it is the base class for sensor board and motor controller Arduino board
'''
class SerialPort(object):
    def __init__(self):
        self.ComDict = {}
        self.ser = serial.Serial()
    
    def set_port(self,  port,  rate, parity="NONE"):
        par_dict = {
            "ODD": serial.PARITY_ODD,
            "EVEN": serial.PARITY_EVEN,
            "NONE": serial.PARITY_NONE
        } 
        self.ser.port = port
        self.ser.baudrate = rate
        self.ser.byteSize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.parity = par_dict[parity]#serial.PARITY_NONE
        self.ser.timeout = 0.001
        # self.ser.dsrdtr = False
        # self.ser.xonxoff = False
        # self.ser.rtscts = False
    
    def get_port(self):
        if self.ser.isOpen():
            return self.ser.port
        else:
            print("not open port")
            return None
        
    def detect_port(self):
        self.ComDict={}
        # 1, get all port 
        port_list  = list(serial.tools.list_ports.comports()) 
        print(port_list)
        #2, clear all port box
#        self.video_com_box.clear()
        #3, show the insert port com
        for port in port_list:
            self.ComDict["%s" % port[0]] = "%s" % port[1]
            print(port[0],port[1]) 
#            self.video_com_box.addItem(port[0])
        #4, error handle, msgbox show
        if len(self.ComDict) == 0:
            print("no com port!")

    def get_port_com(self, str):
        for key in self.ComDict:
            if self.ComDict[key] == str:
                return key
        print("not found!")
        return None
    
    def init_port(self,  port,  rate):
        # print(port,rate)
        # print("111:", self.ser.is_open)
        self.set_port(  port,  rate )
        if ( not self.ser.is_open ):
            self.open_port()
            self.flush()
                
    def open_port(self):
        if (not self.ser.is_open ):
            # print("222")
            # print(self.ser.is_open)
            self.ser.open()
#            return None
    
    def close_port(self):
        # print(self.ser.is_open)
        if self.ser.is_open :
            self.ser.close()
    
    def flush(self):
        self.ser.read(50)
    
    def read_data(self):
        # data=''
        try:
            data = self.ser.readline()
            # print("ser read: ",  data)
            data = data.decode()
        except:
            data=""
        return data
    
    def send_data(self,  data):
        if self.ser.isOpen():
            self.ser.write( data )
            # print("send data:"+ str(data))

    def match_data(self, data, pat, groupkey):
        m_d = ""
        pattern = re.compile(pat)
        m = pattern.match(data)
        if m:
            m_d = m.group(groupkey)
        return m_d

    #生成CRC16-MODBUS校验码
    def crc16Add(self,read):  
        crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        data = read.replace(" ", "") #消除空格
        readcrcout = hex(crc16(unhexlify(data))).upper()
        str_list = list(readcrcout)
        # print(str_list)
        if len(str_list) == 5:
            str_list.insert(2, '0')  # 位数不足补0，因为一般最少是5个
        crc_data = "".join(str_list) #用""把数组的每一位结合起来  组成新的字符串
        # print(crc_data)
        read = read.strip() + ' ' + crc_data[4:] + ' ' + crc_data[2:4] #把源代码和crc校验码连接起来
        # print('CRC16:'+ crc_data[4:] + ' ' + crc_data[2:4])
        # print(read)
        crc_res = crc_data[4:] + crc_data[2:4]
        return read,crc_res

    # 十六进制显示
    def hexShow(self, argv):
        try:
            result = ''
            hLen = len(argv)
            # print(hLen)
            for i in range(hLen):
                hvol = argv[i]
                hhex = '%02x' % hvol
                result += hhex+' '

            # logging.info('Led Read:%s', result)
            print('hexShow:',result)
            return result
        except Exception as e:
            print("---异常---：", e)
            return ""

    def hexShow_2(self, argv):
        try:
            result = ''
            hLen = len(argv)
            # print(hLen)
            for i in range(hLen):
                hvol = ord(argv[i])
                hhex = '%02x' % hvol
                result += hhex+' '

            # logging.info('Led Read:%s', result)
            # print('hexShow: '+ result)
            return result
        except Exception as e:
            print("---异常---：" + e)
            return ""

    def read_data2(self, BOOL=1):
        # while BOOL:
        print(self.ser.in_waiting)
        # print(self.se)
        if (self.ser.in_waiting > 0):
            readbuf = self.ser.read(self.ser.in_waiting)
            return readbuf
            # self.hexShow(readbuf)

            
def test_serialport():
    ser = SerialPort()
    #1, detect port
    ser.detect_port()
    # find port
    port_com = ser.get_port_com("USB Serial")

    #2, set port
    ser.set_port(  port_com , 38400)

    #3, open port
    ser.open_port()
    print(ser.get_port())
    # 4. flush data
    ser.flush()
    #5. send check cmd
    read_cmd = "01 03 00 01 00 01 D5 CA"
    # bytes_cmd = bytes(read_cmd.split(" "))
    # print(bytes_cmd)
    # hex_cmd = bytes_cmd
    # read_cmd = "010300010001D5CA"
    hex_cmd =  bytes.fromhex(read_cmd)
    # ser.send_data( read_cmd.decode("hex") )
    print(hex_cmd)
    ser.send_data( hex_cmd )
    #6. read data
    # data = ser.read_data2()
    print(ser.ser.inWaiting())
    data = ser.ser.read(10)
    ser.hexShow(data)
    result = ser.hexShow(data[:-2])
    # 7. crc16 check data
    read, crc_res = ser.crc16Add(result)
    print(data[-2:].hex())
    print(crc_res.lower())
    if crc_res.lower() == data[-2:].hex():
        print("check right")
    # print("2")
    # print(data)
    #5. data process
# #    data = data.split("sz|su")
#     print(data)
#     data = re.split("sz|su|\n",  data )
#     print("data:")
#     print(data[1:-1])
    
#    pass

def test_read():
    ser = SerialPort()
    print(ser.ser.port)
    port = "/dev/ttyUSB1"
    rate = 9600
    ser.init_port(port,rate)
    while True:
        print( ser.read_data() )

def test_write():
    ser = SerialPort()
    port = "/dev/ttyUSB0"
    rate = 9600
    ser.init_port(port,rate)
    while True:
        ser.send_data("niubi")
        # print( ser.read_data() )

def test_choice(flag):
    # print(type(flag))
    if flag is '0':
        test_read()
    elif flag is '1':
        # print("write")
        test_write()

if __name__=="__main__":
    test_serialport()
    # test_read()
    # try: 
    #     flag = sys.argv[1]
    # except:
    #     flag = 0
    # flag = sys.argv[1]
    
    # test_choice(flag)
    
