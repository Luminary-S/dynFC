#!/usr/bin/python
# -*- coding: utf-8 -*-
# import ros
import sys,os,time
import math
# import serial 
from serialPort import SerialPort

import threading
# import threading

BOOL = True  
BAUD_ = 38400
RESOLUTION =  65536
Encoder_CMD = {
    1: "01 03 00 01 00 01 D5 CA",
    2: "02 03 00 01 00 01 D5 F9"
    # 1: [0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA],
    # 2: [0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xF9]
}
Encoder_CMD_2 = {
    # 1: "01 03 00 01 00 01 D5 CA",
    # 2: "02 03 00 01 00 01 D5 F9"
    1: [0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA],
    2: [0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xF9]
}

VERSION_ = sys.version
print(VERSION_)

class DualEncoder(SerialPort):

    def __init__(self):
        super(DualEncoder,self).__init__()
        self.read_buf= []
        self.tmp_angle = 0
        self.cnt = 0
        self.d1_init = 0
        self.d2_init = 0
        self.d1 = self.d1_init
        self.d2 = self.d2_init

    def ReadData(self):
        global BOOL

        d1 = self.d1_init
        d2 = self.d2_init
        while BOOL:
            # for i in Encoder_CMD:
            self.cnt +=1
            if self.get_data( Encoder_CMD_2[1] ):
                d1 = self.tmp_angle
            if self.get_data( Encoder_CMD_2[2] ):
                d2 = self.tmp_angle
            print((d1,d2))
            self.read_buf.append( (d1,d2) )  # degree not radian
            if len(self.read_buf) > 100:
                self.read_buf = self.read_buf[-100:]


    def DOpenPort( self, portx, bps):
        ret = False
        try:

            self.set_port(  portx , bps )
            #3, open port
            self.open_port()

            if(self.ser.is_open):
                ret=True
                self.flush()
                self.data_init()
                threading.Thread(target= self.ReadData).start()
                return ret
        except Exception as e:
            print("---error---：", e)    

    def DClosePort(self):
        global BOOL
        BOOL=False
        self.ser.close()

    def get_data(self, CMD):
        #1. send check cmd
        hex_cmd = bytes.fromhex(CMD)
        print(hex_cmd)
        self.send_data( hex_cmd )
        # for i in CMD:
        #     self.ser.write(chr(i))
        #6. read data
        try:
            data = self.ser.read(10)
            print(data)
            print(type(data))
            self.hexShow(data)
            result = self.hexShow(data[:-2])
            # 7. crc16 check data
            # print(result)
            read, crc_res = self.crc16Add(result)
            print(data[-2:].hex())
            print(crc_res.lower())
            if crc_res.lower() == data[-2:].hex():
                d_list = result.split(" ")[-3:-1]
                print(d_list)
                self.tmp_angle = self.get_num(d_list) 
                print("check right")
                return True
        except  Exception as e:
            print("DualEncoder finder error!")
            # self.close_port()
            # return False
        # self.close_port()
        return False

    def get_data_2(self, CMD):
        #1. send check cmd
        # hex_cmd = CMD.decode("hex") #bytes.fromhex(CMD)
        # print(hex_cmd)
        # self.send_data( hex_cmd )
        for i in CMD:
            self.ser.write(chr(i))
        #6. read data
        try:
            data = self.ser.read(10)
            # print(data)
            # print(type(data))
            data_raw = self.hexShow_2(data)
            result = self.hexShow_2(data[:-2])
            # 7. crc16 check data
            # print(result)
            read, crc_res = self.crc16Add(result)
            # print(data_raw.split(" ")[-3:-1])
            # print(crc_res.lower())
            if crc_res.lower() == "".join(data_raw.split(" ")[-3:-1]):
                d_list = result.split(" ")[-3:-1]
                # print(d_list)
                self.tmp_angle = self.get_num(d_list) 
                # print("check right")
                return True
        except  Exception as e:
            print("DualEncoder finder error!")
            # self.close_port()
            # return False
        # self.close_port()
        return False

    def get_num(self, d_list):
        # for d in range(2):
        # print(d_list[0].hex())
        d1 = int( d_list[0], 16 )
        d2 = int( d_list[1], 16 )
        angle = 360.0 * ( d1*256 + d2 ) / RESOLUTION
        # print("angel:",angle)
        # self.read_buf.append(angle)
        return angle
    
    def data_init(self):
        self.d1_init = self.get_data_2( Encoder_CMD_2[1] )
        self.d2_init = self.get_data_2( Encoder_CMD_2[2] )
        self.d1 = self.d1_init
        self.d2 = self.d2_init

    def open_dualencoder( self, portx, bps):
        ret = False
        try:
            # 打开串口，并得到串口对象
            
            self.set_port(  portx , bps )
            
            #3, open port
            self.open_port()
            # print("--++++++")
            #判断是否打开成功
            if(self.ser.is_open):
                ret=True
                self.flush()
                # print("--+++")
                self.data_init()
                # print("--+++++")
                # threading.Thread(target= self.ReadData).start()
                return ret
        except Exception as e:
            print(e) 

    def read_data(self):
        # d1 = self.d1_init
        # d2 = self.d2_init
        
        # for i in Encoder_CMD:
        # self.cnt +=1
        # t0 = time.clock()
        if self.get_data_2( Encoder_CMD_2[1] ):
            self.d1 = self.tmp_angle
        # print(time.clock() - t0)
        # t0 = time.clock()
        if self.get_data_2( Encoder_CMD_2[2] ):
            self.d2 = self.tmp_angle
        # print(time.clock() - t0)
        # print((self.d1,self.d2))
        # t0 = time.clock()
        self.read_buf.append( (self.d1,self.d2) )  # degree not radian
        if len(self.read_buf) > 100:
            self.read_buf = self.read_buf[-100:]
        # print(time.clock() - t0)
        

def test_main():
    # 1. get com port 
    DE = DualEncoder()
    ret = DE.open_dualencoder( "/dev/ttyUSB0", BAUD_)
    DE.read_data()
    # while ret:
    #     if DE.cnt > 104:
    #         DE.DClosePort()
    #         break

if __name__ == "__main__":
    test_main()        