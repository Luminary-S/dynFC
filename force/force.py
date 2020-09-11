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

import sys
import os
# sys.path.append("/home/sgl/catkin_new/src/dynforcecontrol")
sys.path.append(os.path.abspath(os.path.dirname(__file__)+'/'+'..'))
import math
import numpy as np
from ur.ur_node import URNode
from ur.ur_move import UR
from PID import PID
import ur.utility as UT
import rospy

data = [29.999999999999996, 31.25333233564303, 32.48689887164855, 33.68124552684677, 34.81753674101715, 35.87785252292473, 36.84547105928689, 37.70513242775789, 38.44327925502016, 39.0482705246602, 39.510565162951536, 39.82287250728689, 39.98026728428272, 39.98026728428272, 39.82287250728689, 39.51056516295154, 39.0482705246602, 38.44327925502016, 37.705132427757896, 36.8454710592869, 35.877852522924734, 34.817536741017165, 33.68124552684678, 32.486898871648556, 31.25333233564304, 30.000000000000007, 28.74666766435697, 27.513101128351455, 26.318754473153213, 25.182463258982846, 24.122147477075274, 23.15452894071312, 22.294867572242097, 21.556720744979845, 20.951729475339803, 20.489434837048464, 20.177127492713115, 20.019732715717282, 20.019732715717282, 20.17712749271311, 20.489434837048464, 20.951729475339814, 21.556720744979838, 22.29486757224209, 23.154528940713114, 24.122147477075263, 25.182463258982835, 26.3187544731532, 27.513101128351458, 28.746667664356956, 29.999999999999993, 31.25333233564303, 32.48689887164855, 33.68124552684677, 34.81753674101715, 35.87785252292473, 36.84547105928689, 37.70513242775789, 38.44327925502016, 39.0482705246602, 39.510565162951536, 39.82287250728689, 39.98026728428272, 39.98026728428272, 39.82287250728689, 39.51056516295154, 39.0482705246602, 38.44327925502016, 37.705132427757896, 36.8454710592869, 35.877852522924734, 34.817536741017165, 33.68124552684678, 32.486898871648556, 31.25333233564304, 30.000000000000007, 28.74666766435697, 27.513101128351455, 26.318754473153213, 25.182463258982846, 24.122147477075274, 23.15452894071312, 22.294867572242097, 21.556720744979845, 20.951729475339803, 20.489434837048464, 20.177127492713115, 20.019732715717282, 20.019732715717282, 20.17712749271311, 20.489434837048464, 20.951729475339814, 21.556720744979838, 22.29486757224209, 23.154528940713114, 24.122147477075263, 25.182463258982835, 26.3187544731532, 27.513101128351458, 28.746667664356956]

# print(len(data))
# sys.exit(0)

class URForce(UR):

    def __init__(self, ur_type):
        # super(URForce, self).__init__(self, ur_type, ur_ip=0)
        UR.__init__(self, ur_type, ur_ip=0)
        self.p_rate = 100
        Ffrequency = 5
        Cfrequency = self.p_rate
        Max = 50
        Min = 30
        typ = 0
        shift = 0
        self.gen_force = UT.generate_typical_signal( Max, Min, Ffrequency, Cfrequency, shift, typ )
        self.gen_pos = UT.generate_typical_signal( 0.2, 0.2, Ffrequency, Cfrequency, shift, typ )

        self.imp_ctr = PID(0,0,0)
        self.vel_ctr = PID(0,0,0)
        self.num = 0

            
    # for constant data using =======================
    # def get_F_target(self):
    #     data = [29.999999999999996, 31.25333233564303, 32.48689887164855, 33.68124552684677, 34.81753674101715, 35.87785252292473, 36.84547105928689, 37.70513242775789, 38.44327925502016, 39.0482705246602, 39.510565162951536, 39.82287250728689, 39.98026728428272, 39.98026728428272, 39.82287250728689, 39.51056516295154, 39.0482705246602, 38.44327925502016, 37.705132427757896, 36.8454710592869, 35.877852522924734, 34.817536741017165, 33.68124552684678, 32.486898871648556, 31.25333233564304, 30.000000000000007, 28.74666766435697, 27.513101128351455, 26.318754473153213, 25.182463258982846, 24.122147477075274, 23.15452894071312, 22.294867572242097, 21.556720744979845, 20.951729475339803, 20.489434837048464, 20.177127492713115, 20.019732715717282, 20.019732715717282, 20.17712749271311, 20.489434837048464, 20.951729475339814, 21.556720744979838, 22.29486757224209, 23.154528940713114, 24.122147477075263, 25.182463258982835, 26.3187544731532, 27.513101128351458, 28.746667664356956, 29.999999999999993, 31.25333233564303, 32.48689887164855, 33.68124552684677, 34.81753674101715, 35.87785252292473, 36.84547105928689, 37.70513242775789, 38.44327925502016, 39.0482705246602, 39.510565162951536, 39.82287250728689, 39.98026728428272, 39.98026728428272, 39.82287250728689, 39.51056516295154, 39.0482705246602, 38.44327925502016, 37.705132427757896, 36.8454710592869, 35.877852522924734, 34.817536741017165, 33.68124552684678, 32.486898871648556, 31.25333233564304, 30.000000000000007, 28.74666766435697, 27.513101128351455, 26.318754473153213, 25.182463258982846, 24.122147477075274, 23.15452894071312, 22.294867572242097, 21.556720744979845, 20.951729475339803, 20.489434837048464, 20.177127492713115, 20.019732715717282, 20.019732715717282, 20.17712749271311, 20.489434837048464, 20.951729475339814, 21.556720744979838, 22.29486757224209, 23.154528940713114, 24.122147477075263, 25.182463258982835, 26.3187544731532, 27.513101128351458, 28.746667664356956]
    #     data = [39.99999999999999, 41.25333233564303, 42.48689887164855, 43.68124552684677, 44.81753674101715, 45.87785252292473, 46.84547105928689, 47.70513242775789, 48.44327925502016, 49.0482705246602, 49.510565162951536, 49.82287250728689, 49.98026728428272, 49.98026728428272, 49.82287250728689, 49.51056516295154, 49.0482705246602, 48.44327925502016, 47.705132427757896, 46.8454710592869, 45.877852522924734, 44.817536741017165, 43.68124552684678, 42.486898871648556, 41.253332335643044, 40.00000000000001, 38.74666766435697, 37.51310112835145, 36.31875447315321, 35.18246325898285, 34.12214747707527, 33.15452894071312, 32.2948675722421, 31.556720744979845, 30.951729475339803, 30.489434837048464, 30.177127492713115, 30.019732715717282, 30.019732715717282, 30.17712749271311, 30.489434837048464, 30.951729475339814, 31.556720744979838, 32.29486757224209, 33.154528940713114, 34.12214747707526, 35.182463258982835, 36.3187544731532, 37.51310112835146, 38.746667664356956, 39.99999999999999, 41.25333233564303, 42.48689887164855, 43.68124552684677, 44.81753674101715, 45.87785252292473, 46.84547105928689, 47.70513242775789, 48.44327925502016, 49.0482705246602, 49.510565162951536, 49.82287250728689, 49.98026728428272, 49.98026728428272, 49.82287250728689, 49.51056516295154, 49.0482705246602, 48.44327925502016, 47.705132427757896, 46.8454710592869, 45.877852522924734, 44.817536741017165, 43.68124552684678, 42.486898871648556, 41.253332335643044, 40.00000000000001, 38.74666766435697, 37.51310112835145, 36.31875447315321, 35.18246325898285, 34.12214747707527, 33.15452894071312, 32.2948675722421, 31.556720744979845, 30.951729475339803, 30.489434837048464, 30.177127492713115, 30.019732715717282, 30.019732715717282, 30.17712749271311, 30.489434837048464, 30.951729475339814, 31.556720744979838, 32.29486757224209, 33.154528940713114, 34.12214747707526, 35.182463258982835, 36.3187544731532, 37.51310112835146, 38.746667664356956]
    #     F_init = np.array( [ -3.65, -81.7, 49.75, -1.675, 0.728, -2.364 ] )
    #     Fd = np.array( [0]* 6 )
    #     iid = self.num % 100
    #     Fd[2] = data[iid] # python2 is next, python3 is __next__()
    #     self.num = self.num + 1
    #     print("Fd 2: ", Fd[2])
    #     Fd = Fd #+ F_init
    #     return Fd

   
    def vel_controller(self, target, now, gain):
        # gain = np.array(gain) #np.array([0,0,1,0,0,0])
        Gain = np.diag(gain)
        # print("Gain: ", Gain)
        # vd = np.array( [[0]*6] )
        interval = 1.0 / self.p_rate
        target = np.array(target)
        now = np.array(now)
        v_des = ( target -  now ).reshape(6,1)
        print("v_des: ", v_des.T)
        vd = v_des / interval - np.dot( Gain, v_des ) 
        return list( vd.T ) 

    def get_F_target(self, F_init):
        # F_init = np.array( [ -3.65, -81.7, 49.75, -1.675, 0.728, -2.364 ] )
        Fd = np.array( [0]* 6 )
        # iid = self.num % 100
        Fd[2] = self.gen_force.next() # python2 is next, python3 is __next__(), z axis
        # Fd[0] = 0  # x axis
        # Fd[1] = 0  # y axis
        self.num = self.num + 1
        print("Fd 2: ", Fd[2])
        Fd = Fd #+ F_init
        return Fd

    def force_part(self, F, Fd, q):
        control_flag = [ 0, 0, 1, 0, 0, 0 ]

        delat_F = Fd[2] - F[2]
        gain = 1.0/803
        vz = -delat_F * gain
        x_dot_FS = [-vz,0,0,0,0,0]
        # x_dot_FS = x_dot_force_FS #+ x_dot_clean_FS 
        print_round(x_dot_FS,"X_dot_FS: ")
        return x_dot_FS

    def force_controller(self, F, Fd, q):
        # 1. force part
        f_x_dot_FS = self.force_part(F,Fd,q)
        # 2. position part
        # p_x_dot_FS = self.pos_controller(target,now)
        # 3. xdot target in force sensor frame
        x_dot_FS = np.array(f_x_dot_FS)
        # 4. trans to ee frame
        qdot = self.qdot_from_VFS(x_dot_FS)
        return qdot

    def get_pos_target(self, pos_init, rot_init, q):
        # pos_init = np.array( [ -3.65, -81.7, 49.75, -1.675, 0.728, -2.364 ] )
        # pos_init = 
        pos_init = pos_init.reshape(1,3)
        # p = np.array( [0,0,0] )
        # iid = self.num % 100
        num = self.gen_pos.next() # x axis
        # print("gen target: ", num)
        p= np.array( [num,0,0] )
        # err = np.append(p, np.array([0,0,0]))

        FS_T_targetFS = UT.Rp2T( UT.RPY2Mat( [0,0,0] ),  np.array( p ) )
        # target_T_FS = np.identity(4)
        # print("FS_T_target: ", FS_T_targetFS)
        ee_T_FS = self.transFS2ee()
        targetFS_T_targetee = UT.InvT(ee_T_FS)  # rigid body, T is inverse
        ee_T_targetee = np.dot( np.dot(  ee_T_FS,  FS_T_targetFS ), targetFS_T_targetee )
        pd =  UT.transl(ee_T_targetee).reshape(1,3) # + pos_init
        # print(ee_T_targetee)
        # base_T_ee = self.crBot.MFK(q)  # q in radians, T:4*4
        # base_T_targetee = np.dot( base_T_ee, ee_T_targetee)
        # print(base_T_targetee)
        # sys.exit(0)
        # print("p: ", p)
        # p[1] = 0  # y axis
        # p[2] = 0  # z axis
        # pd = p + pos_init
        # print("init: ", pos_init)
        # print("pd: ", pd)
        
        return pd, rot_init

    def pos_part(self, target, now, gain):
    # def vel_controller(self, target, now, gain):
        # error_in_ee = 
        x_dot_FS = self.vel_controller(target, now, gain)
        # x_dot_FS = x_dot_force_FS #+ x_dot_clean_FS 
        # print_round(x_dot_FS,"X_dot_FS: ")
        x_dot = np.array( [ x_dot_FS[0][0], 0, 0,0,0,0] )
        print("x_dot_FS: ", x_dot)
        return x_dot
    
    def pos_controller(self, target, now, q):
        # move down, x axis in FS frame
        # target = [0.2,  0, 0,  0, 0, 0]
        # now =    [0,  0, 0, 0, 0, 0]
        gain =   [99.25, 0, 0, 0, 0, 0]
        p_FS_xdot_cleanT = self.pos_part(target,now,gain)
        FS_xdot_cleanT = np.array(p_FS_xdot_cleanT)
        qdot = self.ur_ee_to_base_move( FS_xdot_cleanT, q ) 
        return qdot

    def qdot_from_VFS(self, FS_xdot_cleanT, q):
        # force sensor frame ( clockwise rotate 45 degree with z axis):   
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
        ee_T_FS = self.transFS2ee()
        ee_of_cleanee_jac_FS_of_cleanT = UT.tr2jac( ee_T_FS ) # samebody adjoint matrix
        print("ee_jac_FS: ", ee_of_cleanee_jac_FS_of_cleanT)
        print("FS_xdot_cleanT: ", FS_xdot_cleanT)
        # inv_jac_ee = np.linalg.pinv(jac_ee)
        ee_xdot_cleanee = np.dot( ee_of_cleanee_jac_FS_of_cleanT, FS_xdot_cleanT.T )
        print("ee_xdot_cleanee: ", ee_xdot_cleanee)
        qdot = self.ur_ee_to_base_move( ee_xdot_cleanee, q ) 
        # qdot = self.ur_vel_move( np.array(x_dot_FS), q )
        return qdot

    def controller(self, target, now, F,Fd,q):
        # 1. force part
        f_x_dot_FS = self.force_part(F,Fd,q)
        # 2. position part
        target = [0,  -0.2, 0, 0, 0, 0]
        now = [0,  0, 0, 0, 0, 0]
        gain = [0,  100, 0, 0, 0, 0]
        p_x_dot_FS = self.pos_part(target,now,gain)
        # 3. xdot target in force sensor frame
        x_dot_FS =  np.array(p_x_dot_FS) + np.array(f_x_dot_FS) 
        # 4. trans to ee frame
        qdot = self.qdot_from_VFS(x_dot_FS)
        return qdot

class ForceNode(URNode):

    def __init__(self):
        super(ForceNode,self).__init__()
        self.force = np.array( [[0]*6] )
        self.forceMove = URForce("ur3")

    def get_param(self):
        pass

    def set_init(self):
        q = self.ur.now_q
        self.F_init = self.get_now_force()
        rot, pos = self.forceMove.get_now_rot_pos(q)
        self.rot_init = rot
        self.pos_init = pos

    def get_now_force(self):
        return self.force

    def get_now_rot_pos(self,q):
        base_T_init = UT.Rp2T(self.rot_init, self.pos_init)
        rot_now, pos_now = self.forceMove.get_now_rot_pos(q)
        base_T_now = UT.Rp2T(rot_now, pos_now)
        init_T_now = np.dot( UT.InvT(base_T_init), base_T_now )
        T = init_T_now
        return UT.tr2r(T), UT.transl(T)
    
    def get_T(self,q):
        T = self.forceMove.get_forward_T(q)
        return T

    def go_to_pos(self, q):
        qd = [i/180*math.pi for i in q]
        self.set_moveType("fj")
        self.ur_move_to_point(self.joint_pub,qd)

    def update_(self):
        q = self.ur.now_q
        # # 1. generate target, only z axis
        # Fd = self.forceMove.get_F_target()
        Pd, Rd = self.forceMove.get_pos_target(self.pos_init, self.rot_init, q)
        # # print("Fd: ",Fd)
        # print_round(Fd, "Fd: ")
        # 2. get now force and pos
        # F = self.get_now_force()
        R, P = self.get_now_rot_pos(q)

        target = np.append( Pd.reshape(1,3), np.array( [0]*3 ) )
        now = np.append( P.reshape(1,3), np.array( [0]*3 ) )
        print("init: ", self.pos_init.T)
        print("target: ", target)
        print("now: ", now)
        # now = 
        # # print("F: ",F)
        # print_round(F,"F: ")
        # sys.exit(0)

        # qdot = self.forceMove.force_controller(F, Fd, q)
        qdot = self.forceMove.pos_controller(target, now, q)
        # qdot = [round(i,3) for i in qdot]
        print("qdot: ", qdot)
        # sys.exit(0)
        self.urscript_speedj_pub(self.joint_pub, qdot, 0.5, 0.5)
        # vzd = list( np.array(Fd) - np.array(F) )[2]
        # self.vel_zd_pub.publish(vzd)

    def spin(self):
        rate = self.Rate(self.forceMove.p_rate)
        q_init = [-6.736, -17.769, 86.226, 283.793, 80.197, 270.892]
        self.go_to_pos(q_init)
        rospy.sleep(3)
        i = 0
        DEBUG = 0
        self.set_init()
        while not self.is_shutdown(): 
            i += 1  
            if DEBUG:   
                if i ==  100:
                    break  
            print("-----------loop %s start------------" %i)
            self.update_()
            print("-----------loop end-----------")
            rate.sleep()
        print("---end all---")


    def spin2(self):
        rate = self.Rate(self.forceMove.p_rate)
        q_init = [-38.45, -37.84, 114.24, 280.19, 66.54, 268.49]
        self.go_to_pos(q_init)
        rospy.sleep(3)
        while not self.is_shutdown():
            print("-----------------------")
            # 1. generate target, only z axis
            Fd = self.forceMove.get_F_target()
            # Fd = 0.0
            # print("Fd: ",Fd)
            print_round(Fd, "Fd: ")
            # 2. get now force
            F = self.get_now_force()
            # print("F: ",F)
            print_round(F,"F: ")
            # 3. get impedace velocity
            # x_dot_FS = self.forceMove.imp_controller( F, Fd )
            # gain = [0,0, -99.991,0,0,0]
            # x_dot_force_FS = self.forceMove.vel_controller( Fd, F, gain )
            delat_F = Fd[2] - F[2]
            gain = 1.0/803
            vz = delat_F * gain
            x_dot_FS = [-vz,0,0,0,0,0]
            # x_dot_FS = x_dot_force_FS #+ x_dot_clean_FS 
            print_round(x_dot_FS,"X_dot_FS: ")
            
            # 5. get joint speed
            q = self.ur.now_q
            qdot = list(self.forceMove.ur_vel_move( np.array(x_dot_FS),q ))
            qdot = [round(i,3) for i in qdot]
            print_round(qdot,"qdot: ")
            self.urscript_speedj_pub(self.joint_pub, qdot, 0.5, 0.5)
            vzd = delat_F
            self.vel_zd_pub.publish(vzd)
            print("-----------end-----------")
            rate.sleep()
    
def print_round(l,name):
    print(name, [round(i,3) for i in l])

# def test():
    # fNode = ForceNode()
    # fNode.init("force_node")
    # fNode.spin()

if __name__ == '__main__':
    # test()
    fNode = ForceNode()
    fNode.init("force_node")
    fNode.spin()
    # fNode.spin2()