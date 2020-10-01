#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy
import math
import copy
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
        if name == "ur5":
            a = [0, 0, -0.42500, -0.39225, 0, 0]
            alpha = [0, math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2]
            d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
        elif name == "ur3":  # modified
            a = [0, 0, -0.24365, -0.21325, 0, 0]
            alpha = [0, math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2]
            d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
        self.A = a
        self.ALPHA = alpha
        self.D = d
    
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
    
    def Mat2AxisAngle(self, R):
        acosinput = (numpy.trace(R) - 1) / 2.0;
        if acosinput >= 1:
            return numpy.array([0.0, 0.0, 0.0])
        elif acosinput <= -1:
            if numpy.linalg.norm(1 + R[2, 2]) > 1e-6:
                omg = (1 / numpy.sqrt(2 * (1 + R[2, 2]))) * numpy.array([R[0, 2], R[1, 2], 1 + R[2, 2]])
            elif numpy.linalg.norm(1 + R[1, 1]) > 1e-6:
                omg = (1 / numpy.sqrt(2 * (1 + R[1, 1]))) * numpy.array([R[0, 1], 1 + R[1, 1], R[2, 1]])
            else:
                omg = (1 / numpy.sqrt(2 * (1 + R[0, 0]))) * numpy.array([1 + R[0, 0], R[1, 0], R[2, 0]])
            so3mat = self.VecToso3(numpy.pi * omg)
        else:
            theta = numpy.arccos(acosinput)
            so3mat = theta / 2.0 / numpy.sin(theta) * (R - R.transpose())
        print('Axis and Angle', theta, so3mat)
        return self.so3ToVec(so3mat)
    
    def AxisAngle2Mat(self, omgtheta):
        theta = numpy.linalg.norm(omgtheta)
        if theta < 1e-6:
            return numpy.eye(3)
        else:
            so3mat = self.VecToso3(omgtheta)
            omgmat = so3mat / theta
            return numpy.eye(3) + numpy.sin(theta) * omgmat + (1 - numpy.cos(theta)) * numpy.dot(omgmat, omgmat)
    
    def VecToso3(self, omg):
        return numpy.array([[ 0, -omg[2], omg[1]], [ omg[2], 0, -omg[0]], [ -omg[1], omg[0], 0]])
    
    def so3ToVec(self, so3mat):
        return numpy.array([so3mat[2, 1], so3mat[0, 2], so3mat[1, 0]])
    
    def InvT(self, T):
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        ans = numpy.zeros((4, 4))
        ans[0:3, 0:3] = R.transpose()
        ans[0:3, 3] = -numpy.dot(R.transpose(), p)
        ans[3,3] = 1
        return ans
    
    # modify DH method (Creig`s book)
    def A1(self, theta, d):
        ans = numpy.array([[+numpy.cos(theta), -numpy.sin(theta), 0, 0],
                           [+numpy.sin(theta), +numpy.cos(theta), 0, 0],
                           [                0,                 0, 1, d],
                           [                0,                 0, 0, 1]])
        return ans
    
    def A2(self, alpha, a):
        ans = numpy.array([[1,                 0,                 0, a],
                           [0, +numpy.cos(alpha), -numpy.sin(alpha), 0],
                           [0, +numpy.sin(alpha), +numpy.cos(alpha), 0],
                           [0,                 0,                 0, 1]])
        return ans
    
    # modify DH method (Creig's book)
    #
    # i-1         i         
    #  +----------+  Oi
    #             |         i+1
    #             +----------+  Qi+1  
    #                       
    def MDH(self, a, alpha, d, theta):
        return numpy.dot(self.A2(alpha, a), self.A1(theta, d))

    def SDH(self, a, alpha, d, theta):
        return numpy.dot(self.A1(alpha, a), self.A2(theta, d))
    
    def MFK(self, theta, id=-1):
        if id == -1:
            id = self.JOINT_SIZE
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(id):
            if self.JOINT_TYPE[k] == 0:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], self.THETA[k]+theta[k]))
            else:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k]+theta[k], self.THETA[k]))
        return T
    
    def MDK(self, theta, id=-1):
        if id == -1:
            id = self.JOINT_SIZE
        Te = self.MFK(theta, id)
        T = numpy.zeros((4, 4))
        J = numpy.zeros((6, id))
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(0, id):
            if self.JOINT_TYPE[k] == 0:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], self.THETA[k]+theta[k]))
                J[0:3, k] = numpy.cross(T[0:3, 2], Te[0:3, 3] - T[0:3, 3])
                J[3:6, k] = T[0:3, 2]
            else:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k]+theta[k], self.THETA[k]))
                J[0:3, k] = T[0:3, 2]
        return J
    
    def FunOriErrAR(self, Rc, Rt):
        Re = numpy.dot(Rc.transpose(), Rt)
        e = 0.5 * numpy.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        eo = numpy.dot(Rc, e)
        return eo
    
    def MIK(self, Rt, Pt, q, iterate_times = 50):
        q_ans = copy.deepcopy(q)
        # print("q_ans: ", q_ans)
        Tc = self.MFK(q)
        # print("pre Tc: ", Tc)
        # print("Rt,Pt: ",Rt,Pt)
        dv = Pt - Tc[0:3, 3]
        dw = self.FunOriErrAR(Tc[0:3, 0:3], Rt[0:3, 0:3])
        count = 0
        while (numpy.linalg.norm(dv) > 1e-5 or numpy.linalg.norm(dw) > 1e-3) and count < iterate_times:
            J = self.MDK(q)
            if abs(numpy.linalg.matrix_rank(J)) < 6:
                print('singularity')
                return q_ans
            
            dx = numpy.array([dv[0], dv[1], dv[2], dw[0], dw[1], dw[2]])
            dq = numpy.dot(numpy.linalg.pinv(J), dx) * 0.5
            
            # methord 1 set robot to 6 dof
            if self.JOINT_SIZE > 6:
                dq[6] = 0
            q = q + dq.flatten()
            # print(q)
            Tc = self.MFK(q)
            dv = Pt - Tc[0:3, 3]
            dw = self.FunOriErrAR(Tc[0:3, 0:3], Rt[0:3, 0:3])
            count = count + 1
        
        print('iterates ', count, 'times')
        if count >= iterate_times:
            print('iterates more than ' + str(iterate_times) + ' times')
            return q
        return q

    def get_adimitance_control_state(self,force):
        k = [[], [], [], [], [], []]
        v_max = [0, 0, 0, 0, 0, 0]
        C = [0, 0, 0, 0, 0, 0]
        v = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            v[i] = self.admintance_control_bspline(force[i], v_max[i], k[i, 1], k[i, 2], k[i, 3], k[i, 4], C)

        
    def admintance_control_bspline(self, f, v_max, k1, k2, k3, k4, C):
        v = 0
        if f < -k4:
            v = -v_max
        elif f < -k3:
            v = -v_max + C * numpy.power(f + k4, 3)
        elif f < -k2:
            v = 3 * C * numpy.power(k2 - k1, 2) * (f + k2) - C * numpy.power(k2 - k1, 3)
        elif f < -k1:
            v = C * numpy.power(f + k1, 3)
        elif f < k1:
            v = 0
        elif f < k2:
            v = C * numpy.power(f - k1, 3)
        elif f < k3:
            v = 3 * C * numpy.power(k2 - k1, 2) * (f - k2) - C * numpy.power(k2 - k1, 3)
        elif f < k4:
            v = v_max + C * numpy.power(f - k4, 3)
        else:
            v = v_max

        return v

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
    print("---------------- curi robotics  -----------")
    # q = [-0.6293776671039026, -0.4619410673724573, 2.021991729736328, 4.664813041687012, 1.202303171157837, 4.682912826538086]
    q = [-0.6987722555743616, 0.05523943901062012, 1.3865890502929688, 4.805746078491211, 1.1122870445251465, 4.619902610778809]
    q = [ -14.08, -7.83, 34.95, 337.5, 86.08, 272.3]
    q = [-28.7,-32.22, 97.69, 299.45, 71.63, 271.10] 
    q_inrad = [ i/ 180.0 * math.pi for i in q]
    q_inrad =  [-0.09881097475160772, -0.17853910127748662, 1.080275058746338, 5.2465314865112305, 1.4176548719406128, 4.72550630569458]
    print(q_inrad)
    # q = [-0.5009072462665003, -0.5624502340899866, 1.704935073852539, 5.22642707824707, 1.250244140625, 4.731635093688965]
    T = crBot.MFK(q_inrad)
    print("T: ", T)
    q_D = [ i*180/math.pi for i in q_inrad ]
    print("joint pos in degree: ", q_D)
    print("roll(x), pitch(y), yaw(z) :", toDegree( crBot.Mat2RPY(T) ) )
    print("pt in base frame: ", T[0:3,3])
    # print("inverse T: ", crBot.InvT(T))
    # print(crBot.RPY2Mat([ 1.0/6*math.pi, 1.0/3*math.pi, 1.0/4*math.pi]))
    # print(crBot.RPY2Mat2([ 1.0/6*math.pi, 1.0/3*math.pi, 1.0/4*math.pi]))

if __name__ == "__main__":
    test()