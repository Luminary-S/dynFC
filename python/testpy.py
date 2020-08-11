#!/usr/bin/python3
# -*- coding: utf-8 -*-
#author: sgl
#date: 20200805
import time
from functools import wraps
import logging

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
	
test() 	

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

foo()