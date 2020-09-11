#!/usr/bin/python
#coding=utf-8
########################################################################
#
# Copyright (c) 2020, CUHK CURI 
# Author: SGL
#
# All rights reserved.
# hybrid force control node
#
#
########################################################################

import sys,os
# sys.path.append("/home/sgl/catkin_new/src/dynforcecontrol")
sys.path.append(os.path.abspath(os.path.dirname(__file__)+'/'+'..'))
import rospy,time,math,yaml
import numpy as np

from ur.ur_node import URNode
from PID import PID

class ForceNode(URNode):

    def __init__(self):
        super(ForceNode,self).__init__()
        self.force  = [0.0,0.0,0.0,0.0,0.0,0.0] # [fx,fy,fz,mx,my,mz]
 
        self.now_ur_pos = [1.6308979988098145, -2.6233118216144007, -2.4940224329577845, 5.343937873840332, -1.7317011992083948, -1.645034138356344]
        # self.ur = UR(0)
        self.now_vel = [0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
        self.imp_ctr = PID(0,0,0)
        self.vel_ctr = PID(0,0,0)
        self.stain_ctr = PID(0,0,0)

    def init(self):
        self.init_node('ur_demo_node')
        self.loginfo("start UR demo control node...")
        self.define_node_publisher()
        self.define_node_subscriber()
    
    def define_node_publisher(self):
        self.joint_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
        self.vel_z_pub = rospy.Publisher("/vz", Float32, queue_size = 1)
        self.vel_y_pub = rospy.Publisher("/vy", Float32, queue_size = 1)
        self.vel_zd_pub = rospy.Publisher("/vzd", Float32, queue_size = 1)
        self.vel_yd_pub = rospy.Publisher("/vyd", Float32, queue_size = 1)
        self.Fzd_pub = rospy.Publisher("/Fzd", Float32, queue_size = 1)
    
    # def define_node_subscriber(self):
    #     # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
    #     joint_sub = rospy.Subscriber("/joint_states", JointState, self.callback_joint)
    #     force_sub = rospy.Subscriber("/robotiq_ft_sensor", ft_sensor, self.callback_ft_sensor, queue_size=1)
    #     # imgPer_sub = rospy.Subscriber('/imgPercent', Float32, self.callback_percent)

    def impedance_part(self, Fd=-17, error_pre = 44):
        stiffness = 1291
        self.imp_ctr.set_pid_params(1.0/stiffness, 0.0000000041, 0.0003)
        v = [0,0,0,0,0,0]
        # lambda_z = 1.33
        # error_pre = 41.1 #35  #  pre force at EE        
        F = self.force[2]
        now_F = F - error_pre
        delta_F = Fd-now_F
        vz_d = self.imp_ctr.update(now_F, Fd)

        if vz_d > 0.1:
            vz_d = 0.1
        elif vz_d < -0.1:
            vz_d = -0.1
        if abs(vz_d) < 0.002:
            vz_d = 0.0
        # vz_d = -0.001
        
        v[0] = -vz_d
        print("vz_d:", -vz_d)
        v= np.mat(v)
        # v=[0,0,0,0,0,0]
        # v[0]:  neg: back; pos: forward        v= np.mat(v)
        # v[1]:  neg: right; pos: lef        v= np.mat(v)
        # v[2]:  neg: down; pos: up        v= np.mat(v)
        # v[3]:  neg: right; pos: lef        v= np.mat(v)
        # v[4]:  neg: right; pos: lef        v= np.mat(v)
        # v[5]:  neg: right; pos: lef        v= np.mat(v)
        return v, delta_F, F

      
    def traj_part(self, xyz_list=[0,1,0]):
        # force sensor frame:   
        #       y ^   
        #         |  
        #         |  
        #   z <---o
        #        /
        #       /
        #      v  x
        v = [0,0,0,0,0,0] 
        # vd = K_d * delta_delta_stain + K_p * delta_stain + K_i * stain
        self.vel_ctr.set_pid_params(0.01, 0.00001,0.0001)
        vy_d = self.vel_ctr.update(0.1, 0.2)
        vy_d = 0.02
        
        # vd = 
        v[2] = -vy_d
        print("vy_d:",-vy_d)
        v = np.mat(v)
        return v
        # pass
    
    def stain_part(self, stain):
        stain_d = 0.001
        # 17, 25
        self.stain_ctr.set_pid_params(7, 0, 0, 0.5)
        Fd = self.stain_ctr.update( stain, stain_d) - 1.0
        # Fd = -Fd
        if Fd < 0.0  and Fd < -4.0:
            Fd = -4.0
        elif Fd >= 0.0:
            Fd = 0.0
        return Fd

    def controller(self, v):
        # vd = K * delta_F
        v0 = [0,0,0,0,0,0]
        '''
        ee coodinate(for RCR):
        v[0] positive: right (people view, forward to wall)
        v[1] positive: window back
        v[2] negative: up  positive: down
        v[3] positive: pitch up (yang); negative: pitch down (fu)
        v[4] positive: roll right (right gundong); negative: roll left (left gundong)
        v[5] positive: yaw clockwise (right turn); negative: yaw counter-clockwise (left turn)
        '''
        delta_v1 = np.mat(v0)
        delta_v2 = np.mat(v0)
        delta_v1, delta_F, F = self.impedance_part()
        print("delta_F:", delta_F)
        if F > 30:
            delta_v2 = np.mat([ abs(delta_F) * 0.0005, 0, 0, 0, 0, 0 ])
        elif abs(delta_F ) < 4:
            # print("in it")
            # delta_v2 = np.mat([ abs(delta_F) * 0.001, 0, 0, 0, 0, 0 ])
        # else:
            delta_v2 = self.traj_part() 
        v = v + delta_v1 + delta_v2
        # v = np.mat(v) 
        return v
    
    # def get_interest_stain(self):
    #     #1. always get the downside of the image size: 10*300

    def get_joint_v(self, v):
        q = self.now_ur_pos
        # T = self.ur.get_T()
        T = self.ur.set_T(q)
        Jac = self.ur.getJac(q)
        inv_J = np.mat(Jac).I
        # print("inv J:", inv_J)
        qdot_d = np.dot( inv_J, v.T ).T
        qdot_d = qdot_d.tolist()[0]


        return qdot_d
    
    def get_now_ee_v(self):
        q = self.now_ur_pos
        # T = self.ur.get_T()
        # T = self.ur.set_T(q)
        Jac = self.ur.getJac(q)
        qdot_now = np.mat(self.now_vel).T
        v_now = np.dot(Jac, qdot_now).T.tolist()[0]
        print("now_q_vel:", self.now_vel)
        print("v_now:", v_now)
        return v_now

    def spin_demo_move(self):
        rate = self.Rate(self.rate)
        pub = self.joint_pub
        v = [0,0,0,0,0,0]
        v0 = [0,0,0,0,0,0]
        v = np.mat(v)
        time.sleep(0.5)
        #1. go to init position
        # init_pos = [-20.61,-42.064,94.494,-44.189,81.542,260.085] #[23.06, -42.48, -54.97, -70.41, -117.10, 271.66]
        init_pos = [ -38.31, -46.74, -61.73, -69.24, -56.02, 267.31 ]
        # init_pos = [ -1.51,-45.03,-38.51,-98.71,-90.57,267.68 ]
        self.go_init_position( init_pos, 6 )
        tmp_force = self.force[2]
        i = 0
        while i < 5:
            i = i + 1
            tmp_force = tmp_force + self.force[2]
        self.INIT_FORCE = tmp_force / 5
        # time.sleep(10)
        #2. impedance contact
        # self.go_xyz([0,0,0.1]) # forward
        num = 0
        status = "0000"
        Fd = -15
        stain = self.imgPer
        # cv2.imwrite("")
        avg_stain = stain
        vyd = 0.0 
        vzd = 0.0
        checker = 0
        flag = 0
        urpos0 = self.ur.get_pos()
        while not self.is_shutdown():
            # velocity impedance + velocity y
            # vd = self.controller2(v)
            if status == "1000":
                print("======1000======================================================")
                urpos = self.ur.get_pos()
                delta_pos_y = abs( urpos[2] - urpos0[2])
                # print("delta_pos:", delta_pos_y)
                if delta_pos_y  > 0.225: 
                    status = "0000"
                    self.set_stop_clean()
                    continue
                delta_v1 = np.mat(v0)
                delta_v2 = np.mat(v0)
                # Fd = -15
                delta_v1, delta_F, F_sensor = self.impedance_part(Fd, self.INIT_FORCE)
                print("delta_F:", delta_F)
                # print("Fd:", Fd)
                # if F_sensor > 35:
                #     delta_v2 = np.mat([ abs(delta_F) * 0.0005, 0, 0, 0, 0, 0 ])
                # elif abs(delta_F ) < 4:
                if abs(delta_F ) < 4:
                    # print("in it")
                    # delta_v2 = np.mat([ abs(delta_F) * 0.001, 0, 0, 0, 0, 0 ])
                # else:
                    delta_v2 = self.traj_part() 
                    # num = num + 1
                v = delta_v1 + delta_v2
                vyd = v.tolist()[0][2]
                vzd = v.tolist()[0][0]
                print("final_v====:", v)
                # urpos = self.ur.get_pos()
                qdot_d = self.get_joint_v(v)
                self.urscript_speedj_pub(pub, qdot_d, 0.5, 0.5)
                
            elif status == "2000": # cleaning init 
                print("======2000==========================================================")
                # pos = [ 0, -71.49, 38.18, -148.0, -89.79, 270.0]  
                pos = [0.59, -32.97, -49.31, -98.70, -90.07, 268.11]#[0.17, -78.88, 49.33, -149.68, -89.77, 268.29]#[ 0.0, -74.38, 42.07, -147.21, -89.75, 268.39] #[ 0, -69.70, 35.45, -145.27, -89.76, 268.38]#[ 0, -62.85, 24.84, -141.0, -89.78, 268.38]              
                # pos = [-0.05, -68.31, 39.06, -148.91, -87.40, 266.92] #######1. taijn, 2, gaodu, up, 3, jiaoguaban
                self.go_init_position(pos,10)
                # self.set_moveType("sjd")
                # self.ur_move_to_point(pub ,pos)
                num = num + 1
                urpos0 = self.ur.get_pos()
                status = "1000"
                self.set_open_clean()
            elif status == "0000": # check cleaning 
                print("======0000========================================================")
                pos = [-0.79, -33.92, -126.22, -18.59, -89.12, 268.29]#[-2.21, -33.27, -130.62, -14.84, -87.7, 268.29]#[ -0.30, -119.21, 119.79, -178.76, -89.86, 267.45]#[ -0.35, -121.49, 115.50, -172.19, -89.79, 267.09]
                # pos = [2.36, -121.75, 118.49, -174.96, -89.79, 267.89] #### 1, tiaogao 2,
                stain_Threshold = 0.003
                if checker == 0:
                    self.go_init_position(pos, 10)
                new_stain = self.imgPer
                checker = checker + 1
                print("=======checker:====", checker)
                if checker < 5 and new_stain > stain_Threshold:
                    stain = stain + new_stain
                    avg_stain = stain *1.0 / checker                    
                elif new_stain <= stain_Threshold and flag > 1:
                    print("cleaning times:", num)
                    # break
                elif new_stain <=0.005:
                    flag = flag + 1
                    print("flag:", flag)
                else:
                    Fd = self.stain_part(avg_stain)
                    # Fd= -3.0
                    status = "2000"
                    checker = 0
                    flag = 0
                    stain = 0
            print("now stain:", new_stain, "avg_stain:",avg_stain)
            print("Fd:", Fd)

            self.pub_msg(vyd, vzd, Fd)

            rate.sleep()
    
    def pub_msg(self, vyd, vzd,Fd):
        v_now = self.get_now_ee_v()
        self.vel_y_pub.publish(v_now[2])
        self.vel_z_pub.publish(v_now[0])
        self.vel_yd_pub.publish(vyd)
        self.vel_zd_pub.publish(vzd)
        self.Fzd_pub.publish(Fd)

    def set_open_clean(self):
            # def set_clean(self):
        self.set_param("/rcr/clean/",1)
        self.set_param("/rcr/pumpcleanrotate/", 1)
        self.set_param("/rcr/pumpsewagerotate", 1)
    def set_stop_clean(self):
        self.set_param("/rcr/cleanstop", 1)
        self.set_param("/rcr/pumpcleanstop", 1)
        self.set_param("/rcr/pumpsewagestop/", 1)

if __name__=="__main__":
    ur_robot = PathController()
    ur_robot.init()
    ur_robot.spin_demo_move()