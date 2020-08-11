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
import math
import numpy as np
# import rospy
from dynforcecontrol.msg import rcr, sensorArduino, ft_sensor
from frame_node import FrameNode

def SET_PARAMS(self):
    pass

class Force():

    def __init__(self):
        super().__init__()
    
    def set_params(self):
        pass



    """
    generate a force target
    """
    def generate_dyn_force_target(self, Max, Min, frequency, pNum, type=1):
        # linear force generator
        while True:
            if type == 1:  # 20 * sin ( 2*pi*frequency * x ) + 20
                response = yield 20 * math.sin( 2 * math.pi * frequency *cursor )
            elif type == 2:
                response = yield 
            if cursor == pNum:
                cursor = 1
            if response:
                cursor = int(response)
            else:
                cursor += 1
            
        f_range = range(0, 100)
        # 
        dyn = f_range
        return dyn

    def force_controller(self, f_now, f_tar):
        delta_f = f_tar - f_now
        x_dot = self.lamda * math.exp(delta_f)
        return x_dot
    
    # def arm

    def PID(self):
        pass

    def Compensator(self):
        pass

class ArmControl():

    def __init__(self):
        super().__init__()

    def xdot2qdot(self):
        pass

class ForceNode(FrameNode):

    def __init__(self):
        super(ForceNode).__init__()
        self.force = np.array( [[0]*6] )
        self.forceNode = FrameNode()

    def get_now_force(self):
        return self.force

    """
    ros node
    """
    def force_node(self):
        # self.pub = rospy.Publisher('/corva', Float32, queue_size=1 )
        pass

    
    def spin(self):
        rate = self.Rate(20)

        # n=0
        while not self.is_shutdown():
            print("-----------------------")
            # l = self.update()
            print("-----------end-----------")
            rate.sleep()

def test():
    force = Force()
    
    # 1. generate force targe
    dyn_target = force.generate_dyn_force_target()

    # 2. get now force from sensor
    #  

    pass


if __name__ == '__main__':
    test()