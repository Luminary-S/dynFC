#!/usr/bin/python
# coding=utf-8
########################################################################
#
# Copyright (c) 2020, CUHK CURI
# Author: SGL
#
# All rights reserved.
# force control class for UR
#
#
########################################################################

import math,sys,os
import numpy as np
sys.path.append(os.path.abspath(os.path.dirname(__file__)+'/'+'..'))
from ur.ur_node import URNode
from ur.ur_move import UR
from PID import PID
import ur.utility as UT
import rospy


# sys.path.append("/home/sgl/catkin_new/src/dynforcecontrol")


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
        self.gen_force = UT.generate_typical_signal(
            Max, Min, Ffrequency, Cfrequency, shift, typ)
        self.gen_pos = UT.generate_typical_signal(
            0.2, 0.2, Ffrequency, Cfrequency, shift, typ)

        self.imp_ctr = PID(0, 0, 0)
        self.vel_ctr = PID(0, 0, 0)
        self.num = 0

    def vel_controller(self, target, now, gain):
        # gain = np.array(gain) #np.array([0,0,1,0,0,0])
        Gain = np.diag(gain)
        # print("Gain: ", Gain)
        # vd = np.array( [[0]*6] )
        interval = 1.0 / self.p_rate
        target = np.array(target)
        now = np.array(now)
        v_des = (target - now).reshape(6, 1)
        print("v_des: ", v_des.T)
        vd = v_des / interval - np.dot(Gain, v_des)
        return vd

    def get_F_target(self, F_init):
        # F_init = np.array( [ -3.65, -81.7, 49.75, -1.675, 0.728, -2.364 ] )
        Fd = np.array([0] * 6)
        # iid = self.num % 100
        # python2 is next, python3 is __next__(), z axis
        Fd[2] = self.gen_force.next()
        # Fd[0] = 0  # x axis
        # Fd[1] = 0  # y axis
        self.num = self.num + 1
        print("Fd 2: ", Fd[2])
        Fd = Fd  # + F_init
        return Fd

    def force_part(self, F, Fd, gain):
        delat_F = np.array(Fd) - np.array(F)
        # gain = 1.0/803
        # vz = -delat_F * gain
        # x_dot_FS = [-vz, 0, 0, 0, 0, 0]
        # gain = [0, 0, 0, 99.95, 99.95, 99.95]
        temp_x_dot_FS = - \
            np.array(self.vel_controller([0, 0, 0, 0, 0, 0], delat_F, gain))
        x_dot_FS = self.value_from_gain(temp_x_dot_FS, gain)
        return list(x_dot_FS)

    def force_controller(self, F, Fd, q):
        # 1. force part
        f_x_dot_FS = self.force_part(F, Fd, q)
        # 2. position part
        # p_x_dot_FS = self.pos_controller(target,now)
        # 3. xdot target in force sensor frame
        x_dot_FS = np.array(f_x_dot_FS)
        # 4. trans to ee frame
        eeVee = self.eeVee_from_VFS(x_dot_FS)
        qdot = self.ur_ee_to_base_move(eeVee, q)
        return qdot

    def get_pos_target(self, pos_init, rot_init, q):
        # pos_init = np.array( [ -3.65, -81.7, 49.75, -1.675, 0.728, -2.364 ] )
        # pos_init =
        pos_init = pos_init.reshape(1, 3)
        # p = np.array( [0,0,0] )
        # iid = self.num % 100
        num = self.gen_pos.next()  # x axis
        # print("gen target: ", num)
        p = np.array([num, 0, 0])
        # err = np.append(p, np.array([0,0,0]))

        FS_T_targetFS = UT.Rp2T(UT.RPY2Mat([0, 0, 0]),  np.array(p))
        # target_T_FS = np.identity(4)
        # print("FS_T_target: ", FS_T_targetFS)
        ee_T_FS = self.transFS2ee()
        targetFS_T_targetee = UT.InvT(ee_T_FS)  # rigid body, T is inverse
        ee_T_targetee = np.dot(
            np.dot(ee_T_FS,  FS_T_targetFS), targetFS_T_targetee)
        pd = UT.transl(ee_T_targetee).reshape(1, 3)  # + pos_init
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
        temp_x_dot_ee = self.vel_controller(target, now, gain)
        # x_dot_FS = x_dot_force_FS #+ x_dot_clean_FS
        # print_round(x_dot_FS,"X_dot_FS: ")
        x_dot_ee = self.value_from_gain(temp_x_dot_FS, gain)
        return list(x_dot_ee)

    def pos_controller(self, target, now, q):
        # move down, x axis in FS frame
        # target = [0.2,  0, 0,  0, 0, 0]
        # now =    [0,  0, 0, 0, 0, 0]
        gain = [99.25, 0, 0, 0, 0, 0]
        p_FS_xdot_cleanT = self.pos_part(target, now, gain)
        FS_xdot_cleanT = np.array(p_FS_xdot_cleanT)
        qdot = self.ur_ee_to_base_move(FS_xdot_cleanT, q)
        return qdot

    def value_from_gain(self, x_dot, gain):
        flag = np.array([1 if i != 0 else 0 for i in gain])
        return np.multiply(x_dot, flag)

    def eeVee_from_VFS(self, FS_xdot_cleanT):
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
        ee_T_FS = self.transFS2ee()
        ee_of_cleanee_jac_FS_of_cleanT = UT.tr2jac(
            ee_T_FS)  # samebody adjoint matrix
        # print("ee_jac_FS: ", ee_of_cleanee_jac_FS_of_cleanT)
        # print("FS_xdot_cleanT: ", FS_xdot_cleanT)
        # inv_jac_ee = np.linalg.pinv(jac_ee)
        ee_xdot_cleanee = np.dot(
            ee_of_cleanee_jac_FS_of_cleanT, np.array(FS_xdot_cleanT).T)
        # print("ee_xdot_cleanee: ", ee_xdot_cleanee)

        # qdot = self.ur_vel_move( np.array(x_dot_FS), q )
        return ee_xdot_cleanee

    # regulation control
    def ori_part(self, F_error, F_error_pre):
        f_dot_error = np.array(F_error) - np.array(F_error_pre)
        gain = [0, 0, 0, 99.95, 99.95, 99.95]
        temp_x_dot_FS = - \
            np.array(self.vel_controller(
                [0, 0, 0, 0, 0, 0], f_dot_error, gain))
        x_dot_FS = self.value_from_gain(temp_x_dot_FS, gain)
        return list(x_dot_FS)

    def hybrid_ori_controller(self, F, F_error, F_error_pre, target, now, q):

        # 1. force part
        Fd = [0, 0, 30, 0, 0, 0]
        f_gain = [0, 0, 1.0/803, 0, 0, 0]
        f_x_dot_FS = self.force_part(F, Fd, f_gain)
        f_x_dot_FS = np.array([[0]*6])
        print("force _x_dot_FS: ", f_x_dot_FS)
        f_eeVee = self.eeVee_from_VFS(f_x_dot_FS)
        print("force_eeVee: ", f_eeVee)
        # 2. position part
        target = self.get_pos_target(F)
        p_gain = [99.25, 0, 0, 0, 0, 0]
        # change frame in target setting
        p_x_dot_ee = self.pos_part(target, now, p_gain)
        p_x_dot_ee = np.array([[0]*6])
        print("pos_x_dot_ee: ", p_x_dot_ee)

        # 3. ori part
        o_gain = [0, 0, 0, 99.95, 99.95, 99.95]
        o_x_dot_FS = self.ori_part(F_error, F_error_pre, o_gain)
        print("orientation_x_dot_FS: ", o_x_dot_FS)
        o_eeVee = self.eeVee_from_VFS(o_x_dot_FS)
        o_eeVee = np.array([[0]*6])
        print("orientation_eeVee: ", o_eeVee)
        x_dot_FS = np.array(f_eeVee) + np.array(p_x_dot_ee) + np.array(o_eeVee)
        qdot = self.ur_ee_to_base_move(ee_xdot_ee, q)
        return q_dot


class ForceNode(URNode):

    def __init__(self):
        super(ForceNode, self).__init__()
        self.force = np.array([[0]*6])
        self.forceMove = URForce("ur3")
        self.status = "100"

    def get_param(self):
        pass

    def set_init(self):

        q = self.now_ur_pos
        self.F_init = self.get_now_force()
        rot, pos = self.forceMove.get_now_rot_pos(q)
        self.rot_init = rot
        self.pos_init = pos
        self.pre_F = self.F_init
        self.F_error_pre = np.array([[0]*6])
        # self.F_error =  np.array([[0]*6])

    def get_now_force(self):
        return self.force

    def get_now_rot_pos(self, q):
        base_T_init = UT.Rp2T(self.rot_init, self.pos_init)
        rot_now, pos_now = self.forceMove.get_now_rot_pos(q)
        base_T_now = UT.Rp2T(rot_now, pos_now)
        init_T_now = np.dot(UT.InvT(base_T_init), base_T_now)
        T = init_T_now
        return UT.tr2r(T), UT.transl(T)

    def get_T(self, q):
        T = self.forceMove.get_forward_T(q)
        return T

    def go_to_pos(self, q):
        qd = [i/180*math.pi for i in q]
        self.set_moveType("fj")
        self.ur_move_to_point(self.joint_pub, qd)

    def update_ori_hybrid(self):
        # 1. get now force and force error , and now pos
        F = self.get_now_force()
        F_error = np.array(self.pre_F) - np.array(F)
        F_error_pre = self.F_error_pre
        q = self.now_ur_pos
        R, P = self.get_now_rot_pos(q)
        Pd, Rd = self.forceMove.get_pos_target(self.pos_init, self.rot_init, q)
        now = np.append(P.reshape(1, 3), np.array([0]*3))
        target = np.append(Pd.reshape(1, 3), np.array([0]*3))
        # 2. force part
        qdot = self.forceMove.hybrid_ori_controller(
            F, F_error, F_error_pre, target, now, q)

        print("qdot: ", qdot)
        # sys.exit(0)
        self.pre_F = F
        self.F_error_pre = F_error
        # 4. publish to ur
        self.urscript_speedj_pub(self.joint_pub, qdot, 0.5, 0.5)

    def spin_ori_hybrid(self):
        rate = self.Rate(self.forceMove.p_rate)
        q_init = [-29.61, 16.74, 79.71, 273.34, 64.53, 261.54]
        self.go_to_pos(q_init)
        rospy.sleep(3)
        i = 0
        DEBUG = 0
        self.set_init()
        while not self.is_shutdown():
            i += 1
            if DEBUG:
                if i == 100:
                    break
            print("-----------loop %s start------------" % i)
            self.update_ori_hybrid()
            print("-----------loop end-----------")
            rate.sleep()
        print("---end all---")

    def update_pos(self):
        q = self.now_ur_pos
        # # 1. generate target, only z axis
        # Fd = self.forceMove.get_F_target()
        Pd, Rd = self.forceMove.get_pos_target(self.pos_init, self.rot_init, q)
        # # print("Fd: ",Fd)
        # print_round(Fd, "Fd: ")
        # 2. get now force and pos
        # F = self.get_now_force()
        R, P = self.get_now_rot_pos(q)

        target = np.append(Pd.reshape(1, 3), np.array([0]*3))
        now = np.append(P.reshape(1, 3), np.array([0]*3))
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

    def spin_pos(self):
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
                if i == 100:
                    break
            print("-----------loop %s start------------" % i)
            self.update_pos()
            print("-----------loop end-----------")
            rate.sleep()
        print("---end all---")

    def update_all(self, status):
        F_THRESHOLD = 2
        if status == "100":
            self.update_force()
            if abs(self.F_error_pre[2]) <= F_THRESHOLD:
                self.status = "111"
                self.set_init()
            else:
                self.status = "100"
        elif status == "111":
            self.update_ori_hybrid()
            if abs(self.F_error_pre[2]) <= F_THRESHOLD:
                self.status = "111"
                # self.set_init()
            else:
                self.status = "100"


    def spin_all(self):
        rate = self.Rate(self.forceMove.p_rate)
        q_init = [-29.61, 16.74, 79.71, 273.34, 64.53, 261.54]
        self.go_to_pos(q_init)
        rospy.sleep(3)
        i = 0
        DEBUG = 0
        self.set_init()
        while not self.is_shutdown():
            
            i += 1
            if DEBUG:
                if i == 100:
                    break
            print("-----------loop %s start--, status: %s----------" % (i,self.status))
            # print(self.status)
            self.update_all(self.status)
            print("-----------loop end-----------")
            rate.sleep()
        print("---end all---")

    def update_force(self):
        # 1. get now force and force error , and now pos
        F = self.get_now_force()
        Fd = [0, 0, 30, 0, 0, 0]
        F_error = np.array(self.pre_F) - np.array(F)
        F_error_pre = self.F_error_pre
        q = self.now_ur_pos
        R, P = self.get_now_rot_pos(q)
        # Pd, Rd = self.forceMove.get_pos_target(self.pos_init, self.rot_init, q)
        # now = np.append(P.reshape(1, 3), np.array([0]*3))
        # target = np.append(Pd.reshape(1, 3), np.array([0]*3))
        # 2. force part
        qdot = self.forceMove.force_controller(F, Fd, q)

        print("qdot: ", qdot)
        # sys.exit(0)
        self.pre_F = F
        self.F_error_pre = F_error
        # 4. publish to ur
        self.urscript_speedj_pub(self.joint_pub, qdot, 0.5, 0.5)

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
            print_round(F, "F: ")
            # 3. get impedace velocity
            # x_dot_FS = self.forceMove.imp_controller( F, Fd )
            # gain = [0,0, -99.991,0,0,0]
            # x_dot_force_FS = self.forceMove.vel_controller( Fd, F, gain )
            delat_F = Fd[2] - F[2]
            gain = 1.0/803
            vz = delat_F * gain
            x_dot_FS = [-vz, 0, 0, 0, 0, 0]
            # x_dot_FS = x_dot_force_FS #+ x_dot_clean_FS
            print_round(x_dot_FS, "X_dot_FS: ")

            # 5. get joint speed
            q = self.now_ur_pos
            qdot = list(self.forceMove.ur_vel_move(np.array(x_dot_FS), q))
            qdot = [round(i, 3) for i in qdot]
            print_round(qdot, "qdot: ")
            self.urscript_speedj_pub(self.joint_pub, qdot, 0.5, 0.5)
            vzd = delat_F
            self.vel_zd_pub.publish(vzd)
            print("-----------end-----------")
            rate.sleep()

    def controller_choice(self, ic):
        controller_list = ["force", "pos", "force-pos"]
        ctr = controller_list[ic]
        # q = [-0.6705663839923304, -0.6085761229144495, 2.005333423614502, 5.002155303955078, 1.0891844034194946, 4.865204811096191]
        print("following excute: " + ctr + " control!")
        try:
            if direction == "force":
                self.update_pos()
            elif direction == "pos":
                self.update_force()
            elif direction == "force-pos":
                self.update_force_pos()
        except KeyError as e:
            print("cmd not in list!")

    def spin(self):
        rate = self.Rate(self.forceMove.p_rate)
        q_init = [-27.196, -34.005, 117.804, 277.021, 56.843, 269.964]
        self.go_to_pos(q_init)
        rospy.sleep(3)
        i = 0
        DEBUG = 0

        try:
            print("===in python 3 it is input, in python2 should use raw_input ====")
            inp = raw_input(
                "controller choice(q: quit)? [0,1,2], [force, pos, force-pos: ")
            if inp == "q":
                return
            self.ik_test(int(inp))
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

        self.set_init()
        while not self.is_shutdown():
            i += 1
            if DEBUG:
                if i == 100:
                    break
            print("-----------loop %s start------------" % i)
            self.update_pos()
            print("-----------loop end-----------")
            rate.sleep()
        print("---end all---")


def print_round(l, name):
    print(name, [round(i, 3) for i in l])

# def test():
    # fNode = ForceNode()
    # fNode.init("force_node")
    # fNode.spin()


if __name__ == '__main__':
    # test()
    fNode = ForceNode()
    fNode.init("force_node")
    fNode.spin_all()
    # fNode.spin2()
