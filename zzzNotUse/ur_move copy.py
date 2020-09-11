#!/usr/bin/venv python
# -*- coding: utf-8 -*-

# Copyright (c) 2018.10, Guangli SUN
# All rights reserved.


from ur3_kinematics import *

from jacobian import *
import numpy as np
import numpy.linalg as lg
import math
import matplotlib.pyplot as pl
import sys


class UR():
    def __init__(self, num):
#        self.port = ''
        self.qstart = [ 92.24,-135.31,-99.37, 226.82, -87.84, 3.86 ]#[ 91.48, -202.73, 70.01, 136.88, -88.94, 180.24 ]
        self.ur_num = num
        self.q = self.qstart
        self.ur_status = 'urstop'
        self.weights = [1.] * 6
        self.radius = 0.22
        # weights = [1.] * 6
        self.cont = 15
        self.now_ur_pos = getpi(self.qstart)
        self.now_vel = [0,0,0,0,0,0]
        self.init_q = self.qstart
        self.final_q = self.init_q
        self.ur_init_ready = 0
        self.ur_final_ready = 0
        self.ur_ready = 0
        self.urtype = "ur3"
        self.kin = Kinematic()
        self.jac = ur_robot(self.urtype,6)
        self.STEP = 0.002 # unit: m        
        self.moveType = "stop"
        # self.step_move = {
        #     "up": self.ur_step_move_up,
        #     "down": self.ur_step_move_down,
        #     "back": self.ur_step_move_backward,
        #     "forward": self.ur_step_move_forward,
        #     "left": self.ur_step_move_left,
        #     "right": self.ur_step_move_right
        # }


    """kinematics related """
    def set_UR_ROBOT(self,type):
        self.kin.set_urtype(type)
        self.urtype = type
    
    def get_urobject_urkine(self):
        ur0 = Kinematic()
        return ur0

    def get_IK_from_T(self,T,q_last):
        ur0 = self.kin
        return ur0.best_sol( self.weights, q_last, T )

    def solve_q(self, q, T):
        # rad
        qd = self.kin.best_sol(self.weights, q, T)
        return qd

    def getJac(self,q):
        return self.jac.dk(q)

    """q related """
    def get_qstart(self):
        return self.qstart
    
    def get_q(self):
        print("get_q,",self.now_ur_pos)
        return self.now_ur_pos
    
    def get_T(self):
        return self.kin.Forward(self.now_ur_pos)
    
    def get_pos(self):
        T = self.get_T()
        return [T[3],T[7], T[11]]

    def set_T(self,q):
        return self.kin.Forward(q)
    
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


    def set_step(self, step):
        self.STEP = step

    def ur_step_q(self, q, F_T):
        # step designed joints rotation
        urk = self.kin
        F_T = urk.Forward(q)
        qd = self.solve_q(q, F_T)
        return qd

    def ur_step_move(self, q , i, direction):
        # step cartersian space, end effector movement
        # print(" enter calculation....")
        # print("q:",q)
        urk = self.kin
        F_T = urk.Forward(q)
        print("before F_t:", F_T)
        print("before up and down: ",F_T[11])
        # TransT = self.get_T_translation(F_T)
        print("========step=====", self.STEP)
        F_T[i] = F_T[i] + direction*self.STEP   
        qd = self.solve_q(q, F_T)
        T = self.kin.Forward(qd)
        print("up and down: ",T[11])
        # T = self.
        # print("qd",qd)
        # sys.exit(0)
        return qd

    def ur_step_joint_move(self, q, i, angle):
        delta = self.getpi(angle)
        qd = q
        qd[i] = q[i] + delta 
        return qd

    def ur_step_vel_move(self, vel):
        urdk = self.jac
        vx = 0
        vy = 0
        vz = -vel[i] #vz = v_imp
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
        return v, qdot_d

    def ur_xyz_move(self,q,xyz_list):
        #up:x; left:y; back:z
        [x,y,z] = xyz_list
        urk = self.kin
        F_T = urk.Forward(q)
        F_T[11] = F_T[11] - x        
        F_T[3] = F_T[3] + y
        F_T[7] = F_T[7] - z
        # F_T[i] = F_T[i] + direction*self.STEP  
        qd = self.solve_q(q, F_T)
        return qd

    def ur_xyz_move_backup(self,q,xyz_list):
        #up:x; left:y; back:z
        [x,y,z] = xyz_list
        urk = self.kin
        F_T = urk.Forward(q)
        F_T[11] = F_T[11] - x        
        F_T[3] = F_T[3] + y
        F_T[7] = F_T[7] - z
        # F_T[i] = F_T[i] + direction*self.STEP  
        Rt = self.TtoR(F_T)
        Pt = self.TtoL(F_T)
        # Rt= np.array(Rt)
        # Pt = np.array(Pt)
        # q = np.array(q)
        qd = self.jac.ik(Rt,Pt,q)
        # qd = qd.tolist()
        # qd = self.solve_q(q, F_T)
        return qd

    def TtoR(self, T):
        R = [ [ T[0], T[1], T[2] ], [ T[4], T[5], T[6] ], [ T[8], T[9], T[10] ] ]
        return R
    def TtoL(self, T):
        L = [ T[3], T[7], T[11] ]
        return L

    def ur_step_move_up(self, q):
        print("set direction...")
        return self.ur_step_move(q, 11, -1)
    
    def ur_step_move_down(self, q):
        return self.ur_step_move(q, 11, 1)
    
    def ur_step_move_left(self, q):
        return self.ur_step_move(q, 3, -1)
    
    def ur_step_move_right(self, q):
        return self.ur_step_move(q, 3, 1)
    
    def ur_step_move_forward(self, q):
        return self.ur_step_move(q, 7, -1)
    
    def ur_step_move_backward(self, q):
        return self.ur_step_move(q, 7, 1)
    

    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]

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
            # self.urscripte_speedj_pub(self.pub, qdot_d, 0.5, 0.5)
            # rospy.sleep(0.1)


def getpi(l):
    res = []
    print(l)
    for i in l:
        res.append( i / 180 * math.pi )
    return res

def getDegree(l):
    res = []
    for i in l:
        res.append(  i*180 / math.pi )
    return res

def test_ur():
    qstart = [76.02, -160.79, -77.66, 227.14, -102.12, 264.76]
    q0 = getpi(qstart)
    ur = UR(0)
    qd = ur.ur_step_move(q0, 11,-1)


if __name__=="__main__":
    test_ur()
