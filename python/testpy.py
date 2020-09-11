#!/usr/bin/python2
# -*- coding: utf-8 -*-
#author: sgl
#date: 20200805
import time
from functools import wraps
import logging
import matplotlib.pyplot as plt
import numpy as np
import math

def time_func(func):
    @wraps(func) # use test original attributes
    def func_in(*args,**kwargs):
        t1 = time.time()
        
        
        func(*args,**kwargs)
        t2 = time.time()
        print("Function %s running time: %f" %(func.__name__ ,t2-t1) )
    return func_in
	
@time_func
def test():
	print('无参函数的测试')
	
# test() 	

def use_logging(level):
    def decorator(func):
        def wrapper(*args, **kwargs):
            if level == "warn":
                logging.warn("%s is running" % func.__name__)
            elif level == "info":
                logging.info("%s is running" % func.__name__)
            return func(*args)
        return wrapper

    return decorator

@use_logging(level="warn")
def foo(name='foo'):
    print("i am %s" % name)

# foo()
class FGenerator(object):
    def __init__(self):
        super(FGenerator,self).__init__()
        self.cursor = 0
    
    def generate_dyn_force_target( self, Max, Min, Ffrequency, Cfrequency, shift=0, type=1 ):
        """
        force generator, type 1 sine wave, type 2 sawtooth wave
        params:
         Ffrequency: force frequency, vibration frequency, about 2~5 hz++++++++++++++++++++
         Cfrequency: control frequency, in ros 100hz
         shift: percent of cycle
        """
        # t = 0.0
        # while True:
        #     t = 1 / frequency / pNum * self.cursor
        #     if type == 1:  # sine wave: 20 * sin ( 2*pi*frequency * x ) + 20
        #         response = yield ( (Max-Min)/2 * math.sin( 2 * math.pi * (frequency * t + shift)) + (Max+Min)/2 )
        #     elif type == 2: # Sawtooth Wave
        #         response = yield (Max-Min) * (frequency * t + shift) 
        #     if self.cursor == pNum:
        #         self.cursor = 0
        #     if response:
        #         self.cursor = int(response)
        #     else:
        #         self.cursor += 1
        t = 0.0
        num = 0
        while num < Cfrequency:
            if type == 1:  # sine wave: 20 * sin ( 2*pi*frequency * x ) + 20
                yield ( (Max-Min)/2 * math.sin( 2 * math.pi * (Ffrequency * t + shift)) + (Max+Min)/2 )
            elif type == 2: # Sawtooth Wave
                yield (Max-Min) * (Ffrequency * t + shift)  
            num += 1 
            t = 1.0 / Cfrequency * num           
        # print("cur: %d" % cursor)

    # def __next__(self):
    #     return self.next()
        
    # f_range = range(0, 100)
    # 
    # dyn = f_range
    # return dyn
# def generate_dyn_force_target(self, Max, Min, Ffrequency, perioed, shift=0,type=1):
# # pNum = p_rate / frequency ++++++++++++++++++++
# # shift: percent of cycle
# # linear force generator
#     cursor = 0
#     t = 0.0
#     while True:
#         t = 1 / frequency / pNum * cursor
#         if type == 1:  # sine wave: 20 * sin ( 2*pi*frequency * x ) + 20
#             response = yield ( (Max-Min)/2 * math.sin( 2 * math.pi * (frequency * t + shift)) + (Max+Min)/2 )
#         elif type == 2: # Sawtooth Wave
#             response = yield (Max-Min) * (frequency * t + shift) 
#         if cursor == pNum:
#             cursor = 0
#         if response:
#             cursor = int(response)
#         else:
#             cursor += 1
def draw(x,y):
    plt.xlabel('Force(N)')
    plt.ylabel('loops')
    plt.xlim((0,120))
    plt.ylim((-30,30))
    plt.title("Simple Plot")
    # Text(0.5, 0, 'index')
    plt.legend("force")
    plt.plot(x, y)
    plt.show()

p_rate = 100
perioed = p_rate
Ffrequency = 5
Max = 50
Min = 30
typ = 1
shift = 0
fg = FGenerator()
gen = fg.generate_dyn_force_target( Max, Min, Ffrequency, perioed, shift, typ )
# gen = generate_dyn_force_target(Max,Min, frequency, pNum, typ)

p_l = []

i = 0
LIFETIME = 100
x = np.linspace(0, LIFETIME, 100)
print(x)
while i<LIFETIME:
    # p = gen.__next__()
    p = gen.next()
    p_l.append(p)
    i+=1
print(p_l)
# draw(x,p_l)
