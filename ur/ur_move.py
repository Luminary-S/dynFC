#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) 2018.10, Guangli SUN
# All rights reserved.


# from ur3_kinematics import *

# from jacobian import *
import sys,os
import numpy as np
import numpy.linalg as lg
import math
import matplotlib.pyplot as pl
from curi_robotics import curi_robotics as CRbot
import utility as UT

class UR():
    def __init__(self, ur_type, ur_ip=0):
        self.crBot = CRbot(ur_type,6)
        self.STEP = 0.002 # unit: m

    def set_now_q_and_v(self, q, v):
        self.now_q = q
        self.now_v = v

    def get_now_rot_pos(self,q):
        # q = self.now_q
        base_T_ee = self.crBot.MFK(q)
        # ee_T_FS = self.transFS2ee()
        # base_T_FS = np.dot( base_T_ee, ee_T_FS )
        T = base_T_ee
        return  UT.tr2r(T), UT.transl(T)

    def get_general_jacobian(self, q):
        return self.crBot.MDK(q)

    def get_forward_T(self, q):
        return self.crBot.MFK(q)
    
    def get_inverse_q(self, T, q):
        Rt = T[0:3,0:3]
        Pt = T[0:3,3]
        print("after pt: ", Pt)
        return self.crBot.MIK( Rt, Pt, q)

    def ur_vel_move(self, v, q):
        J = self.crBot.MDK(q)
        J_inv = np.linalg.pinv( J )
        print("J_inv: ", J_inv)
        qdot = np.dot( J_inv, v.T)
        return qdot
    
    def ur_ee_to_base_move(self, ee_v_ee, q):
        # ee_v_ee: 6*1
        base_T_ee = self.crBot.MFK( q )
        base_of_ee_J_ee_of_ee = UT.tr2jac( base_T_ee, 1 )
        print("base_of_ee_J_ee_of_ee: ", base_of_ee_J_ee_of_ee)
        # J_ee_inv = np.linalg.pinv( J_ee )
        base_v_ee = np.dot( base_of_ee_J_ee_of_ee, ee_v_ee)
        print("base_v_ee: ", base_v_ee)
        ee_J_q = self.crBot.MDK( q )
        q_J_ee = np.linalg.pinv( ee_J_q )
        print("inv_base_J_ee: ", q_J_ee)
        qdot  = np.dot( q_J_ee, base_v_ee )
        return qdot

    def sim_ur_ee_to_base_move(self, ee_v_ee, q):
        # ee_v_ee: 6*1
        # base_T_ee = self.crBot.MFK( q )
        # base_of_ee_J_ee_of_ee = UT.tr2jac( base_T_ee, 0 )
        # print("base_of_ee_J_ee_of_ee: ", base_of_ee_J_ee_of_ee)
        # J_ee_inv = np.linalg.pinv( J_ee )
        # base_v_ee = np.dot( base_of_ee_J_ee_of_ee, ee_v_ee)
        # print("base_v_ee: ", base_v_ee)
        ee_J_q = self.crBot.MDK( q )
        q_J_ee = np.linalg.pinv( ee_J_q )
        print("inv_base_J_ee: ", q_J_ee)
        qdot  = np.dot( q_J_ee, ee_v_ee.reshape(6,1) )
        return qdot

    def transFS2ee(self):
        return UT.Rp2T( UT.RPY2Mat( [0, 0, math.pi] ),  np.array( [-0.0395,0,0] ) )

    def ur_xyz_relative_move(self, q, xyz_list):
        #up:x; left:y; back:z
        # ee robot-defined:   
        #         o----> x
        #       / |   
        #      /  |   
        #   y v   v  z
        # base robot-defined: 
        #         o----> x
        #       / |   
        #      /  |   
        #   y v   v  z

        # 1. T_target in ee frame
        # T_target_in_view = np.identity(4)
        # T_target_in_view[0:3, 3] = xyz_list
        # T_target_in_view[0:3, 0:3] = -np.identity(4)
        print(xyz_list)
        FS_T_targetFS = UT.Rp2T( UT.RPY2Mat( [0,0,0] ),  np.array( xyz_list ) )
        # target_T_FS = np.identity(4)
        # print("FS_T_target: ", FS_T_targetFS)
        ee_T_FS = self.transFS2ee()
        # FS_T_ee = UT.Rp2T( UT.RPY2Mat( [0,0, 0] ),  np.array( xyz_list ) )
        # print("ee_T_FS: ", ee_T_FS)
        targetFS_T_targetee = UT.InvT(ee_T_FS)  # rigid body, T is inverse

        # 2. FK get T_base
        base_T_ee = self.crBot.MFK(q)  # q in radians, T:4*4
        print("pt pre: ", base_T_ee[0:3,3])
        # 3. T trans between ee and base
        # T_base_b0 = self.trans_ee_to_base()
        # T_ee_in_base = np.dot( T_base, T_base_b0 )
        # 4. T of target in base frame
        print(np.dot( np.dot(  ee_T_FS,  FS_T_targetFS ), targetFS_T_targetee ))
        base_T_targetee = np.dot( base_T_ee, np.dot( np.dot(  ee_T_FS,  FS_T_targetFS ), targetFS_T_targetee ) ) 
        # print("target T: ", base_T_targetee)
        qd = self.get_inverse_q( base_T_targetee, q)
        print("now q:", q)
        qd = qd.tolist()
        # qd = self.solve_q(q, F_T)
        return qd

    def set_step_move_STEP(self, len):
        self.STEP = len


    def ur_step_move_up(self, q):
        # print("set direction...")
        return self.ur_xyz_relative_move(q , [ 0, 0, 1*self.STEP])
    
    def ur_step_move_down(self, q):
        return self.ur_xyz_relative_move(q , [ 0, 0, -1*self.STEP] )
    
    def ur_step_move_left(self, q):
        return self.ur_xyz_relative_move(q , [0,1*self.STEP,0])
    
    def ur_step_move_right(self, q):
        return self.ur_xyz_relative_move(q , [0,-1*self.STEP,0])
    
    def ur_step_move_forward(self, q):
        return self.ur_xyz_relative_move(q , [1*self.STEP,0,0])
    
    def ur_step_move_backward(self, q):
        return self.ur_xyz_relative_move(q , [-1*self.STEP,0,0])

    
    def get_init_q(self):
        q = getDegree( self.init_q )
        return q
    def set_init_q(self):
        self.init_q = self.now_ur_pos
        return self.init_q
        
    def get_final_q(self):
        q = getDegree(self.final_q)
        return q
    def set_final_q(self):
        self.final_q = self.now_ur_pos
        return self.final_q


    def demo_vel_move(self):
        '''
        ee coodinate:
        v[0] positive: right (people view, forward to wall)
        v[1] positive: window back
        v[2] negative: up  positive: down
        v[3] positive: pitch up (yang); negative: pitch down (fu)
        v[4] positive: roll right (right gundong); negative: roll left (left gundong)
        v[5] positive: yaw clockwise (right turn); negative: yaw counter-clockwise (left turn)
        '''
        ur = ur_robot("ur3", 6)
        

        num = 2
        v0 = 0
        v1 = 0
        l0 = 0
        l1 = 0.12
        vmax = 0.16
        amax = 1
        v_list = []
        t_list = self.t_planning_simple(0, 0.08, v0,v1, vmax,amax)
        # v_list = self.vel_planning_simple(0,0.1,0,0, 0.15)
        print(t_list)
        # t =
        # pl.plot(t, v_list,'-')
        # pl.show()
        # sys.exit(0)
        q_init = [85.85,-117.86,-89.94,200.49,-94.08,3.38]
        # self.ur_movej_to_point(self.pub, getpi(q_init))
        rospy.sleep(5)
        # sys.exit(0)

        t0 = time.time()
        t2 = t0
        # t1 = t0
        # tt = t1 - t2
        qdot_d = [0,0,0,0,0,0]
        while True:
            # v = [0, 0, -0.1, 0, 0,0]
            # t1 = time.time()
            self.urscript_speedj_pub(self.pub, qdot_d, 0.2, 0.5)
            t2_old = t2
            t2 = time.time()
            tt = t2 - t2_old
            delta_t = t2 - t0 + tt
            print("det_t:",delta_t)
            # if delta_t > 0.5:
            #     sys.exit(0)
            v_y = self.set_vel(delta_t,t_list, vmax, amax)
            # sys.exit(0)
            v_list.append(-v_y)
            v = [0,0, -v_y, 0,0,0]
            q = self.get_q()
            # print("q:",q)
            J = ur.dk(q)
            print("J:", J)

            v = np.mat(v)
            inv_J = np.mat(J).I
            # sys.exit(0)
            # print("inv J:", inv_J)
            qdot_d = np.dot( inv_J, v.T ).T
            qdot_d = qdot_d.tolist()[0]
            # print(type(qdot_d))
            print("qdot_d:", qdot_d)
            if v_y == 0:
                break
        t = range(0,len(v_list),1)
        pl.plot(t, v_list,'-')
        pl.show()
    

    
    def vel_move(self, direction, v_list):
        ur = ur_robot("ur3", 6)
        for i in range(0, len(v_list), 0):
            vx = 0
            vy = 0
            vz = -v_list[i] #vz = v_imp
            theta_x = 0
            theta_y = 0
            theta_z = 0
            v = [vx,vy,vz,theta_x,theta_y,theta_z]
            q = self.get_q()
            print("q:", q)
            J = ur.dk(q)
            print("J:", J)
            v = np.mat(v)
            inv_J = lg.inv(J)
            print("inv J:", inv_J)
            qdot_d = np.dot(inv_J, v.T).T
            qdot_d = qdot_d.tolist()[0]
            print(type(qdot_d))
            print("qdot_d:", qdot_d)



# off-line check
def test_ur():
    qstart = [i/180*math.pi for i in [-38.45, -37.84, 114.24, 280.19, 66.54, 268.49] ]
    ur = UR("ur3")
    qd = ur.ur_step_move_up(q0)


if __name__=="__main__":
    test_ur()
