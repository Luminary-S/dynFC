#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import math
# from hand_in_eye_X import *

def generate_typical_signal( Max, Min, Ffrequency, Cfrequency, shift=0, type=0 ):
    """
    force generator: type 0 constant value, type 1 sine wave, type 2 sawtooth wave
    params:
        Ffrequency: force frequency, vibration frequency, about 2~5 hz++++++++++++++++++++
        Cfrequency: control frequency, in ros 100hz
        shift: percent of cycle
    """
    t = 0.0
    num = 0
    # while num < Cfrequency:
    while True:
        if type == 0:
            constantF = ( Max + Min ) / 2.0
            yield constantF
        elif type == 1:  # sine wave: 20 * sin ( 2*pi*frequency * x ) + 20
            yield ( (Max-Min)/2 * math.sin( 2 * math.pi * (Ffrequency * t + shift)) + (Max+Min)/2 )
        elif type == 2: # Sawtooth Wave
            yield (Max-Min) * (Ffrequency * t + shift)  
        num += 1 
        t = 1.0 / Cfrequency * num  

def TfromThetap( theta, p):
    R = RPY2Mat(theta)
    T = Rp2T(R,p)
    return T


# def tr2jac( T, samebody = 1):
#     #T = np.array(T)
#     # R = tr2r(T)
#     # R = T[0:3,0:3]
#     # l = list(T[0:3,3])
#     R = tr2r(T)
#     #jac = np.zeros((6, 6))
#     """
#     jac = [ jac_part1,  jac_part2;
#             jac_part3,  jac_part4;
#                 ]
#     """
#     if samebody==1:
#         jac_part1 = R.T
#         skewp = np.matrix([
#             [0, -l[2], l[1] ],
#             [l[2], 0, -l[0],],
#             [-l[1], l[0], 0]
#         ])
#         jac_part2 = np.dot( skewp, R).T
#         jac_part3 = np.zeros((3,3))
#         jac_part4 = R.T

#     else:
#         jac_part1 = R.T
#         jac_part2 = np.zeros((3,3))
#         jac_part3 = np.zeros((3,3))
#         jac_part4 = R.T
#     # print(jac_part1)
#     # print(jac_part2)
#     # print(jac_part3)
#     # print(jac_part4)
#     # jac_row1 = np.column_stack( (jac_part1, jac_part2) )
#     # jac_row2 = np.column_stack( (jac_part3, jac_part4) )
#     # jac = np.row_stack( (jac_row1, jac_row2) )
#     jac_row1 = np.hstack( (jac_part1, jac_part2) )
#     jac_row2 = np.hstack( (jac_part3, jac_part4) )
#     jac = np.vstack( (jac_row1, jac_row2) )
#     return jac

def tr2jac( T, samebody=1):
    #T = np.array(T)
    R = tr2r(T)
    l = transl(T).T[0]
    # print(l)
    #jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody==1:
        # print("tr2jac 111")
        # jac_part1 = R.T
        # jac_part2 = np.dot( skew( transl(T)), R).T
        # jac_part3 = np.zeros((3,3))
        # jac_part4 = R.T
        jac_part1 = R.T
        skewp = np.array([
            [0, -l[2], l[1] ],
            [l[2], 0, -l[0],],
            [-l[1], l[0], 0]
        ])
        # print(R)
        # print(skewp)
        jac_part2 = np.dot( skewp, R).T
        # print(jac_part2)
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T

    else:
        # print("tr2jac 000")
        jac_part1 = R.T
        jac_part2 = np.zeros((3,3))
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T
    jac_row1 = np.column_stack( (jac_part1, jac_part2) )
    jac_row2 = np.column_stack( (jac_part3, jac_part4) )
    jac = np.row_stack( (jac_row1, jac_row2) )
    return jac

def InvT(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    ans = np.zeros((4, 4))
    ans[0:3, 0:3] = R.transpose()
    ans[0:3, 3] = -np.dot(R.transpose(), p)
    ans[3,3] = 1
    return ans

def Rp2T(R,p):
    ans = np.identity(4)
    # ans = np.matrix(ans)
    ans[0:3, 0:3] = R
    ans[0:3, 3] = p.T
    ans[3,3] = 1
    return ans

# def RPYp2T()

def RotX(theta):
    ans = np.array([[ 1,                 0,                 0],
                        [ 0, +np.cos(theta), -np.sin(theta)],
                        [ 0, +np.sin(theta), +np.cos(theta)]])
    return ans

def RotY(theta):
    ans = np.array([[ +np.cos(theta), 0, +np.sin(theta)],
                        [                 0, 1,                 0],
                        [ -np.sin(theta), 0, +np.cos(theta)]])
    return ans

def RotZ(theta):
    ans = np.array([[ +np.cos(theta), -np.sin(theta), 0],
                        [ +np.sin(theta), +np.cos(theta), 0],
                        [                 0,                 0, 1]])
    return ans

def RPY2Mat(RPY): # [x, y, z] = [roll, pitch, yaw]
    return np.dot(RotZ(RPY[2]), np.dot(RotY(RPY[1]), RotX(RPY[0])))

def Mat2RPY(mat):
    sy = np.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0]);
    if sy > 1e-6:
        x = np.arctan2(+mat[2, 1], +mat[2, 2])
        y = np.arctan2(-mat[2, 0], +sy)
        z = np.arctan2(+mat[1, 0], +mat[0, 0])
    else:
        x   = np.arctan2(-mat[1, 2], +mat[1, 1])
        y = np.arctan2(-mat[2, 0], +sy)
        z   = 0
    return np.array([x, y, z]) # [x, y, z] = [roll, pitch, yaw]


"""
if l is 3*1 , then get 
skew(l) = [ 0, -l(2), l(1)
            l(2), 0 , -l(0)
            -l(1), l(0), 0]
if l is 1*1, then get
skew(l) = [ 0 , -l[0]
            l[0], 0 ]

"""
def skew(l):
    a, b = np.shape(l)
    try:
        if a == 3:
            s = np.array( [ 0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0 ] )
            s = s.reshape((3,3))
            # print "s:", s
            return s
        elif a == 1:
            s = np.array( [ 0, -l[0], l[0], 0])
            s = s.reshape( (2,2) )
            return s
    except:
        print("erro l size!!!  3*1 or 1*1 required!")


def tr2r(T):
    r = [ 0, 1, 2]
    c = [ 0, 1, 2]
    R1 = T[r]
    R = R1[:,c]
    return R

def transl(T):
    r = [3]
    c = [0 , 1, 2]
    l1 = T[:, r]
    l = l1[c]
    return l

# def test_main():
#     pass
# #     # 1, get the X matrix
# #     X = get_ur3_X()
# #     print "rotation:", tr2r(X)
# #     print "transition:", transl(X)
# #     jac = tr2jac(X)
# #     print "jac:", jac

#     # a, b =np.shape(X[:,3])
#     # print X[:,3]
#     # print a,b

#     # 2, get  the  samebody transfer matrix to jacobian matrix
#     #jac = tr2jac(X)
#     #print "jac", jac


# if __name__=="__main__":
#     # test_main()
#     # p_rate = 100
#     Ffrequency = 5
#     Cfrequency = 100
#     Max = 50
#     Min = 30
#     typ = 0
#     shift = 0
#     gen = generate_typical_signal( Max, Min, Ffrequency, Cfrequency, shift, typ )
#     # a= generate_typical_signal()
#     i = 0
#     while i < 1000:
#         fd = gen.next()
#         i +=1
#         print("fd: ", fd)