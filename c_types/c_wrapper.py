import ctypes

demo_c = ctypes.CDLL('demo_c.so')
demo_c.someFunction.argtypes = (ctypes.c_int, ctypes.c_int)

def call_me(a, b):
    global demo_c
    result = demo_c.someFunction(ctypes.c_int(a), ctypes.c_int(b))
    return int(result)

print(call_me(1, 2))