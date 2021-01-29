#!/usr/bin/python3
# -*- coding: utf-8 -*-
from serialPort import SerialPort
import yaml

CONFIG_FILE = "/home/sgl/catkin_new/src/dynforcecontrol/DualEncoder/portCmd.yaml"
PORT_SAVE_FILE = "/home/sgl/catkin_new/src/dynforcecontrol/config/port.yaml"


class PortFinder(SerialPort):

    def __init__(self):
        super(PortFinder,self).__init__()
        self.config_dict = {}
        self.final_com_dict = {}


    def read_config_yaml(self,fname):
        with open(fname) as f:
            self.config_dict = yaml.load(f)
            print(self.config_dict)
        # pass

    def DualEncoderPort(self, port_com):
        BAUD = self.config_dict["DualEncoder"]["baud"]
        CHECK_CMD = self.config_dict["DualEncoder"]["send"]
        #2, set port
        self.set_port(  port_com , BAUD )
        #3, open port
        self.open_port()
        # 4. flush data
        self.flush()
        #5. send check cmd
        hex_cmd =  bytes.fromhex(CHECK_CMD)
        print(hex_cmd)
        self.send_data( hex_cmd )
        #6. read data
        try:
            data = self.ser.read(10)
            print(type(data))
            self.hexShow(data)
            result = self.hexShow(data[:-2])
            # 7. crc16 check data
            read, crc_res = self.crc16Add(result)
            print(data[-2:].hex())
            print(crc_res.lower())
            if crc_res.lower() == data[-2:].hex():
                print("check right")
                return True
        except  Exception as e:
            print("DualEncoder finder error!")
            self.close_port()
            return False
        self.close_port()
        return False

    def IOboardPort(self, port_com):
        name = "IOboard"
        BAUD = self.config_dict[name]["baud"]
        CHECK_CMD = self.config_dict[name]["send"]
        BACK_CMD = self.config_dict[name]["back"]
        self.set_port(  port_com , BAUD )
        self.open_port()
        self.flush()
        print(CHECK_CMD)
        hex_cmd =  bytes.fromhex(CHECK_CMD)
        print(hex_cmd)
        self.send_data( hex_cmd )        
        try:
            data = self.ser.read(10)
            res = self.hexShow(data) 
            print("res",res.lower())
            print("back", BACK_CMD.lower())     

            if res.replace(" ", "").lower() == BACK_CMD.replace(" ", "").lower():
                print("check right")
                return True
        except  Exception as e:
            print("IOboard finder error!")
            self.close_port()
            return False
        self.close_port()
        return False

    def ForcePort(self, port_com):
        # 16bytes: < 0x20> < 0x4e> < LSB_data1> < MSB_data1> ...< LSB_data6> < MSB_data6> < LSB_crc> < MSB_crc>
        name = "Force"
        BAUD = self.config_dict[name]["baud"]
        CHECK_CMD = self.config_dict[name]["send"]
        BACK_CMD = self.config_dict[name]["back"]
        self.set_port(  port_com , BAUD )
        self.open_port()
        self.flush()
        # print(CHECK_CMD)
        # hex_cmd =  bytes.fromhex(CHECK_CMD)
        # print(hex_cmd)
        # self.send_data( hex_cmd )        
        try:
            data = self.ser.read(50)
            res = self.hexShow(data).lower()
            res_l = res.split(" ")
            print(res_l, type(res_l))
            # id_list = []
            id = res_l.index('20')
            print(id)
            # print(type(tmp_id_list),tmp_id_list)
            # for id in range(len(tmp_id_list)):
            print(res_l[id+1])
            if res_l[id+1] == '4e':
                print("===")
                res_tmp_list = data[id: id+16]
                result = self.hexShow(res_tmp_list[:-2])
                # 7. crc16 check data
                read, crc_res = self.crc16Add(result)
                print(res_tmp_list[-2:].hex())
                print(crc_res.lower())
                if crc_res.lower() == res_tmp_list[-2:].hex():
                    print("check right")
                    return True

        except  Exception as e:
            print("Force finder error!")
            self.close_port()
            return False
        self.close_port()
        return False

    def IMUPort(self, port_com):
        return self.matchPort("IMU", port_com)

    def SonicPort(self, port_com):
        parity = self.config_dict["Sonic"]["parity"]
        return self.matchPort("Sonic", port_com, parity)

    def matchPort(self, name, port_com, parity="NONE"):
        # name = "IOboard"
        BAUD = self.config_dict[name]["baud"]
        CHECK_CMD = self.config_dict[name]["send"]
        BACK_CMD = self.config_dict[name]["back"]
        self.set_port(  port_com , BAUD, parity )
        self.open_port()
        self.flush()
        print(CHECK_CMD)
        hex_cmd =  bytes.fromhex(CHECK_CMD)
        print(hex_cmd)
        self.send_data( hex_cmd )        
        try:
            data = self.ser.read(10)
            res = self.hexShow(data) 
            print("res",res.lower())
            print("back", BACK_CMD.lower())     

            if res.replace(" ", "").lower() == BACK_CMD.replace(" ", "").lower():
                print("check right")
                return True
        except  Exception as e:
            print(name + " finder error!")
            self.close_port()
            return False
        self.close_port()
        return False

    def run(self):
        # fname = "/home/sgl/catkin_new/src/dynforcecontrol/DualEncoder/portCmd.yaml"
        # pFinder = PortFinder()
        self.read_config_yaml(CONFIG_FILE)
        self.detect_port()
        flag = 0
        print("\n---DualEncoder---")
        for port in self.ComDict:
            if not port in self.final_com_dict.values():
                # if not pFinder.final_com_dict.__contains__("DualEncoder"):
                print("------", port)
                b = self.DualEncoderPort(port)
                if b == True:
                    self.final_com_dict["DualEncoder"] = port
                    flag = 1
                    break
        if flag == 0:
            print("----DualEncoder not found----")
        else:
            print("----DualEncoder found: ", self.final_com_dict["DualEncoder"] )
            flag = 0

        print("\n---IOboard---")
        for port in self.ComDict:
            if not port in self.final_com_dict.values():
                # if not pFinder.final_com_dict.__contains__("IOboard"):
                print("------", port)
                b = self.IOboardPort(port)
                if b == True:
                    self.final_com_dict["IOboard"] = port
                    flag = 1
                    break
        if flag == 0:
            print("----IOboard not found----")
        else:
            print("----IOboard found: ", self.final_com_dict["IOboard"] )
            flag = 0

        print("\n---Sonic---")
        for port in self.ComDict:
            if not port in self.final_com_dict.values():
                # if not pFinder.final_com_dict.__contains__("IOboard"):
                print("------", port)
                b = self.SonicPort(port)
                if b == True:
                    self.final_com_dict["Sonic"] = port
                    flag = 1
                    break
        if flag == 0:
            print("----Sonic not found----")
        else:
            print("----Sonic found: ", self.final_com_dict["Sonic"] )
            flag = 0

        print("\n---Force---")
        for port in self.ComDict:
            if not port in self.final_com_dict.values():
                # if not pFinder.final_com_dict.__contains__("IOboard"):
                print("------", port)
                b = self.ForcePort(port)
                if b == True:
                    self.final_com_dict["Force"] = port
                    flag = 1
                    break
        if flag == 0:
            print("----Force not found----")
        else:
            print("----Force found: ", self.final_com_dict["Force"] )
            flag = 0

        print(self.final_com_dict)
        # sfname = "/home/sgl/catkin_new/src/dynforcecontrol/config/port.yaml"

        # with open(sfname,"w") as f:
            # yaml.dump(pFinder.final_com_dict, f )
        with open(PORT_SAVE_FILE, 'w', encoding='utf-8') as f:
            yaml.dump(self.final_com_dict, f, default_flow_style=False)

def test_main():
    pFinder = PortFinder()
    # print("---")
    pFinder.run()

if __name__ == "__main__":
    
    test_main()