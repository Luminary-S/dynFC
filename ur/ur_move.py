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
        self.ur_type = ur_type
        self.crBot = CRbot(ur_type,6)
        self.STEP = 0.002 # unit: m

    def set_now_q_and_v(self, q, v):
        self.now_q = q
        self.now_v = v
    
    def set_ur(self, urtype):
        self.ur_type = urtype
        self.crBot.set_robot_param(urtype)
        # self.ur.set_UR_ROBOT(self.urtype)

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
        return list(qdot)

    def transFS2ee(self):
        return UT.Rp2T( UT.RPY2Mat( [0, 0, math.pi] ),  np.array( [-0.0395,0,0] ) )

    def ur_xyz_relative_move(self, q, xyz_list):
        '''
        # force sensor frame :
        #  z <----o
        #       / |
        #      /  |
        #   y v   v  x
        # ee robot-defined:
        #       x ^   ^ y
        #         |  /
        #         | /
        #  z <----o
        # base robot-defined:
        #            ^ y
        #           /
        #          /
        #   x <---o
        #         |
        #         |
        #         v  z

        FS coodinate:
        v[0] positive: right (people view, forward to wall)
        v[1] positive: window back
        v[2] negative: up  positive: down
        v[3] positive: pitch up (yang); negative: pitch down (fu)
        v[4] positive: roll right (right gundong); negative: roll left (left gundong)
        v[5] positive: yaw clockwise (right turn); negative: yaw counter-clockwise (left turn)
        '''
        # 1. T_target in ee frame
        # T_target_in_view = np.identity(4)
        # T_target_in_view[0:3, 3] = xyz_list
        # T_target_in_view[0:3, 0:3] = -np.identity(4)
        # print(xyz_list)
        FS_T_targetFS = UT.Rp2T( UT.RPY2Mat( [0,0,0] ),  np.array( xyz_list ) )
        # target_T_FS = np.identity(4)
        # print("FS_T_target: ", FS_T_targetFS)
        ee_T_FS = self.transFS2ee()
        # FS_T_ee = UT.Rp2T( UT.RPY2Mat( [0,0, 0] ),  np.array( xyz_list ) )
        # print("ee_T_FS: ", ee_T_FS)
        targetFS_T_targetee = UT.InvT(ee_T_FS)  # rigid body, T is inverse

        # 2. FK get T_base
        base_T_ee = self.crBot.MFK(q)  # q in radians, T:4*4
        # print("pt pre: ", base_T_ee[0:3,3])
        # 3. T trans between ee and base
        # T_base_b0 = self.trans_ee_to_base()
        # T_ee_in_base = np.dot( T_base, T_base_b0 )
        # 4. T of target in base frame
        # print(np.dot( np.dot(  ee_T_FS,  FS_T_targetFS ), targetFS_T_targetee ))
        base_T_targetee = np.dot( base_T_ee, np.dot( np.dot(  ee_T_FS,  FS_T_targetFS ), targetFS_T_targetee ) ) 
        # print("target T: ", base_T_targetee)
        qd = self.get_inverse_q( base_T_targetee, q)
        # print("now q:", q)
        qd = list(qd)
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



