#!./venv/python
import sympy as sy
from ur_move import *
# from sensor_frame import *
# import copy
import rospy, time
from jacobian import *
import numpy as np
from trans_methods import *
from demo_singlercr.msg import rcr, sensorArduino

def apply_ics(sol, ics, x, known_params):
    """
    refer: https://vlight.me/2018/05/01/Numerical-Python-Ordinary-Differential-Equations/
    Apply the initial conditions (ics), given as a dictionary on
    the form ics = {y(0): y0, y(x).diff(x).subs(x, 0): yp0, ...},
    to the solution of the ODE with independent variable x.
    The undetermined integration constants C1, C2, ... are extracted
    from the free symbols of the ODE solution, excluding symbols in
    the known_params list.
    """
    free_params = sol.free_symbols - set(known_params)
    eqs = [(sol.lhs.diff(x, n) - sol.rhs.diff(x, n)).subs(x, 0).subs(ics)
            for n in range(len(ics))]
    sol_params = sy.solve(eqs, free_params)

    return sol.subs(sol_params)

class ImpedanceControl():

    def __init__(self):
        # self.impedance_eq = 0
        self.force = 0
        # self.UR = ur
        self.status = 1

    def set_impedance_eq(self, x, F, tim):
        """
        M ddot(x(t)) + C dot(x(t)) + K x(t) = Fe
        """
        M = 0
        C = 0.9
        K = 80
        X = sy.Function('X')
        # eq = M * sy.diff(X, 2) + C * sy.diff( X ) + K * X - Fe
        t,Fe = sy.symbols('t,Fe')
        eq = M * sy.diff(X(t), t, 2) + C * sy.diff( X(t), t ) + K * X(t) - Fe
        if M != 0:
            eq_sol = sy.dsolve(eq, X(t))
            itc = {X(0):x, X(t).diff(t).subs(t, 0): 0, Fe:F, t:tim}
            sol = apply_ics(eq_sol, itc, t, [Fe])
            sol = sol.rhs.subs(Fe, F)  # get the right of the equation, for left use lhs
            sol = sol.subs(t, tim)
        elif C !=0:
            eq_sol = sy.dsolve(eq, X(t))
            itc = {X(0):x}
            sol = apply_ics(eq_sol, itc, t, [Fe])
            sol = sol.rhs.subs(Fe,F) # get the right of the equation, for left use lhs
            sol = sol.subs(t,tim)
        else:
            sol = sy.solve(eq,X(t))[0].subs(Fe,F)
        print(sol)
        return sol


    def impedance_eq_solver(self, F, Fd, pos, tim):
        # itc = {}
        # t, Fe, X = sy.symbols('t,Fe,X')
        # new_pos = self.set_impedance_eq(pos,F, tim)
        # return new_pos
        stiffness = 10000.0
        delta_F = Fd - F
        delta_y = delta_F/ stiffness
        pos = pos + delta_y
        return pos

    def set_path(self):
        T_list = []

    def imp_controller(self, F, q, T, Jac, sonic, v_planning):
        # velocity control
        #1. force to vel
        stiffness = 2200
        lambda_z = 1.33
        delta_T = 0.001
        Fd = -30  # value of sensor metric, not real
        z_d = sonic * 0.001 # transfer unit to mm
        T = np.array(T).reshape((4,4))
        print("T",T)

        # definition is reverse of the ur panel;
        # in this place, if you don't use elastic ee, you should use R*t to get t, and then get y
        # 0.165 is the distance from ee to the brush end surface; 0.050 is the distance from sonic to base center
        z_now = -transl(T).tolist()[1][0] + 0.165 + 0.05 - 0.020

        print("z_now",z_now)

        error_pre = 0
        delta_f = Fd - (F-error_pre) # 15 is the initial force by the weight of the brush

        imp_vxd = delta_f * 1.0 / stiffness
        print("imp_vxd:", imp_vxd)
        delta_z = z_d - z_now
        Z_threshold = 0.002
        if abs(delta_z) < Z_threshold or F > Fd:
            delta_z = 0
        z_d = lambda_z * delta_z
        print("z_d:",z_d)
        vel_d = z_d + imp_vxd
        print("vel_d:", vel_d)

        if vel_d > 0.15:
            vel_d = 0.15
        elif vel_d < -0.15:
            vel_d = -0.15
        #  v (6*1) = J (6*6) * q (6*1)

        # delta_T = 0.001
        # v_list = self.UR.vel_planning(delta_T)
        # vel_d = self.imp_controller()
        v_planning[1] = - vel_d  # motion direction and value are inverse
        v = v_planning
        print("v_planning:",v)

        '''
                ee coodinate:
                v[0] positive: right (people view, forward to wall)
                v[1] positive: window back
                v[2] negative: up  positive: down
                v[3] positive: pitch up (yang); negative: pitch down (fu)
                v[4] positive: roll right (right gundong); negative: roll left (left gundong)
                v[5] positive: yaw clockwise (right turn); negative: yaw counter-clockwise (left turn)
        '''
        v = np.mat(v)
        inv_J = np.mat(Jac).I
        # print("inv J:", inv_J)
        qdot_d = np.dot( inv_J, v.T ).T
        qdot_d = qdot_d.tolist()[0]
        # print(type(qdot_d))
        print("qdot_d:", qdot_d)
        # self.UR.urscripte_speedj_pub(self.UR.pub, qdot_d, 0.5,0.5)
        # if vel_d == 0:
        #     self.status = 2
        if F > 0.8*Fd:
            self.status = 2
        # # print("i:",i)
        # .sleep(0.01)

        return vel_d, qdot_d
        # self.UR.stop(self.UR.pub)
        # sys.exit(0)
    def impedance_torque_controller(self,F_list,q, qdot):
        qdot_d = qdot
        Mx = F_list[3]       
        My = F_list[4]
        Mz = F_list[5]
        Mxd = 0.3
        Myd = 0.3
        stiff_mx = 2
        stiff_my = 2
        stiff_mz = 2
        # adjust last joint for mx, my to positive mx = 0.5, my =0.5
        delta_mx = Mxd - Mx
        delta_my = Myd - My
        if abs(delta_mx) < 0.1 and abs(delta_my) < 0.1:
            delta_q = stiff_mx * (Mxd - Mx) + stiff_my * (Myd - My)
        qdot_d[5] = qdot[5] + delta_q
        return qdot_d
    
    def set_stiff(self, val):
        self.stiffness = val
    
    def set_lambda_z(self, val):
        self.lambda_z = val
    
    def set_Fd(self,val):
        self.Fd = val
    # def set_T()

    def imp_controller_nosonic(self, F, q, T, Jac, v_planning):
        # velocity control
        #1. force to vel
        stiffness = self.stiffness
        lambda_z = self.lambda_z
        delta_T = 0.001
        Fd = self.Fd  # value of sensor metric, not real
        # z_d = sonic * 0.001 # transfer unit to mm
        T = np.array(T).reshape((4,4))
        # print("T",T)

        # print("z_now",z_now)

        error_pre = -11
        # delta_f = abs(Fd - (F-error_pre)) # 15 is the initial force by the weight of the brush
        delta_f = Fd - (F-error_pre)
        print("F",F)
        print("delta_f:",delta_f)
        print("Fd",Fd)
        imp_vxd = delta_f * 1.0 / stiffness
        print("imp_vxd:", imp_vxd)
        vel_d = imp_vxd
        print("vel_d:", vel_d)

        if vel_d > 0.15:
            vel_d = 0.15
        elif vel_d < -0.15:
            vel_d = -0.15

        v_planning[1] =  vel_d  # motion direction and value are inverse
        v = v_planning
        print("v_planning:",v)

        '''
                ee coodinate:
                v[0] positive: right (people view, forward to wall)
                v[1] positive: window back
                v[2] negative: up  positive: down
                v[3] positive: pitch up (yang); negative: pitch down (fu)
                v[4] positive: roll right (right gundong); negative: roll left (left gundong)
                v[5] positive: yaw clockwise (right turn); negative: yaw counter-clockwise (left turn)
        '''
        v = np.mat(v)
        inv_J = np.mat(Jac).I
        # print("inv J:", inv_J)
        qdot_d = np.dot( inv_J, v.T ).T
        qdot_d = qdot_d.tolist()[0]
        # print(type(qdot_d))
        print("qdot_d:", qdot_d)
        # self.UR.urscripte_speedj_pub(self.UR.pub, qdot_d, 0.5,0.5)
        # if vel_d == 0:
        #     self.status = 2
        if abs(delta_f) < 2:
            qdot_d = [0,0,0,0,0,0]
        

        return vel_d, qdot_d


    """ 1 dim force sensor"""
    def update(self, force, Fd, T, t):
        y = T[7]
        print("y",y)
        yd = y
        if force == 255 :
            yd -= 0.02 # forward 2mm
        else:
            # calculate the val of y by solving impedance equations
            yd = self.impedance_eq_solver( force, Fd, y, t)
        T[7] = yd
        print("yd:",yd)
        return T

def main():
    pass

if __name__ == '__main__':
    main()
    # test()