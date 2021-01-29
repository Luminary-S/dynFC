#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy
import math
import copy
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import utility as UT
#####robotics class #####
class curi_robotics():
    # def __init__(self, joint_size, joint_type, a, alpha, d, theta):
    #     self.JOINT_SIZE = joint_size
        # self.A = a
        # self.ALPHA = alpha
        # self.D = d
        # self.THETA = theta
    def __init__(self, name, joint_size=6, joint_type=[]):
        self.NAME = name
        self.JOINT_SIZE = joint_size
        self.set_robot_param(self.NAME)
        self.THETA = [0,0,0,0,0,0]
        if joint_type == []:
            self.JOINT_TYPE = numpy.zeros((joint_size))
        else:
            self.JOINT_TYPE = joint_type
        return
    
    def set_robot_param(self, name ):
        urdfname = "" 
        if name == "ur5":
            urdfname = "/home/sgl/catkin_new/src/dynforcecontrol/urdf/ur5.urdf" 
        elif name == "ur3":  # modified
            urdfname = "/home/sgl/catkin_new/src/dynforcecontrol/urdf/ur3.urdf" 
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "tool0")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        self.kdl_kin = KDLKinematics(robot, "base_link", "tool0")

    def MDK(self, theta, id=-1):
        J = self.kdl_kin.jacobian(theta)
        return J

    def MFK(self, theta, id=-1):
        T = self.kdl_kin.forward(theta)
        return T

    def MIK(self, Rt, Pt, q, iterate_times = 50):
        q = self.kdl_kin.inverse((Pt,Rt),q)
        if q is None:
            print("kdl no inverse")
            return q
        else:
            return q

    # other functions
    def RotX(self, theta):
        ans = numpy.array([[ 1,                 0,                 0],
                           [ 0, +numpy.cos(theta), -numpy.sin(theta)],
                           [ 0, +numpy.sin(theta), +numpy.cos(theta)]])
        return ans
    
    def RotY(self, theta):
        ans = numpy.array([[ +numpy.cos(theta), 0, +numpy.sin(theta)],
                           [                 0, 1,                 0],
                           [ -numpy.sin(theta), 0, +numpy.cos(theta)]])
        return ans
    
    def RotZ(self, theta):
        ans = numpy.array([[ +numpy.cos(theta), -numpy.sin(theta), 0],
                           [ +numpy.sin(theta), +numpy.cos(theta), 0],
                           [                 0,                 0, 1]])
        return ans
    
    def RPY2Mat(self, RPY): # [x, y, z] = [roll, pitch, yaw]
        return numpy.dot(self.RotZ(RPY[2]), numpy.dot(self.RotY(RPY[1]), self.RotX(RPY[0])))
    def RPY2Mat2(self, RPY): # [x, y, z] = [roll, pitch, yaw]
        return numpy.dot(self.RotY(RPY[2]), numpy.dot(self.RotZ(RPY[1]), self.RotX(RPY[0])))
    
    def Mat2RPY(self, mat):
        sy = numpy.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0]);
        if sy > 1e-6:
            x = numpy.arctan2(+mat[2, 1], +mat[2, 2])
            y = numpy.arctan2(-mat[2, 0], +sy)
            z = numpy.arctan2(+mat[1, 0], +mat[0, 0])
        else:
            x   = numpy.arctan2(-mat[1, 2], +mat[1, 1])
            y = numpy.arctan2(-mat[2, 0], +sy)
            z   = 0
        return numpy.array([x, y, z]) # [x, y, z] = [roll, pitch, yaw]
     
    def InvT(self, T):
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        ans = numpy.zeros((4, 4))
        ans[0:3, 0:3] = R.transpose()
        ans[0:3, 3] = -numpy.dot(R.transpose(), p).T
        ans[3,3] = 1
        return ans

def toDegree(rad):
    return [ i*180/math.pi for i in rad ]

def test():
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
    crBot = curi_robotics("ur3",6)
    print("----------------KDL  -----------")
    # q = [-0.6293776671039026, -0.4619410673724573, 2.021991729736328, 4.664813041687012, 1.202303171157837, 4.682912826538086]
    q = [-0.6987722555743616, 0.05523943901062012, 1.3865890502929688, 4.805746078491211, 1.1122870445251465, 4.619902610778809]
    q = [-28.7,-32.22, 97.69, 299.45, 71.63, 271.10] 
    q_inrad = [ i/ 180.0 * math.pi for i in q]
    # q = [-0.09881097475160772, -0.17853910127748662, 1.080275058746338, 5.2465314865112305, 1.4176548719406128, 4.72550630569458]
    q = [-0.11756259599794561, -0.3101275602923792, 1.5049304962158203, 4.953117847442627, 1.3997013568878174, 4.7279558181762695]
    q = [-0.15492231050600225, -0.42622548738588506, 1.9065275192260742, 4.8102569580078125, 1.3624281883239746, 4.71122932434082]
    q = [-0.17590171495546514, -0.4272683302508753, 2.053140640258789, 4.661294460296631, 1.3415879011154175, 4.711563587188721]
    q= [-0.47466785112489873, -0.5934923330890101, 2.056074619293213, 4.834931373596191, 0.9920908808708191, 4.711755275726318]
    T = crBot.MFK(q)
    q_ik = crBot.MIK( T[0:3,0:3], T[0:3,3], q )
    print("q_ik: ", q_ik)
    print("T: ", T)
    q_D = [ round(i*180/math.pi,3) for i in q ]
    print("joint pos in degree: ", q_D)
    print("roll(x), pitch(y), yaw(z) :", toDegree( crBot.Mat2RPY(T) ) )
    print("pt in base frame: ", T[0:3,3])
    # print("TR: ", T[0:3,0:3])
    # print("Tp: ", T[0:3,3])
    # pt = T[0:3,3].T + numpy.array([0.01,0,0])
    # print(pt) 
    # Tt = UT.Rp2T( T[0:3,0:3] , pt.T) 
    # print("Tt:,",Tt)
    # qt = crBot.kdl_kin.inverse(Tt,q)
    # print("res:",qt)
    # print(toDegree(qt))
    # print("inverse T: ", crBot.InvT(T))
    # print(crBot.RPY2Mat([ 1.0/6*math.pi, 1.0/3*math.pi, 1.0/4*math.pi]))
    # print(crBot.RPY2Mat2([ 1.0/6*math.pi, 1.0/3*math.pi, 1.0/4*math.pi]))

if __name__ == "__main__":
    test()