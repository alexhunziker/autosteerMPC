import ctypes
import os

demo_c = ctypes.CDLL("./demo_c.so")
demo_c.someFunction.argtypes = (ctypes.c_int, ctypes.c_int)

demo_simulink = ctypes.CDLL('minimal_simulink_model/minimal_one.so')
demo_simulink.minimal_one_step.argtypes = (ctypes.c_double, ctypes.POINTER(ctypes.c_double))

def call_me(a, b):
    global demo_c
    result = demo_c.someFunction(ctypes.c_int(a), ctypes.c_int(b))
    return int(result)

def call_simulink():
    print("Call simulink...")
    b = ctypes.c_double(0.0)
    print(demo_simulink.minimal_one_initialize())
    demo_simulink.minimal_one_step(ctypes.c_double(8.0), ctypes.byref(b))
    demo_simulink.minimal_one_step(ctypes.c_double(0.0), ctypes.byref(b))
    return b
    
    

print(call_me(1, 2))
print(call_simulink())
