# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/4 15:49
import numpy as np
import math
import matplotlib.pyplot as plt
import time
x0 = 0
z0 = 0

T_sw = 4
T_st = 2
s = 30
h=  30
PI = np.pi


def trajectory_sw(t:float):

    if t>=0 and t<=T_sw:
        x = x0 + s* np.sin(t/T_sw* PI- PI/2)
        z = z0 + h* np.sin(PI - t/T_sw * PI)
    elif t>T_sw <=T_sw+T_st:
        t = t-T_sw
        x = x0 + s * np.sin(t / T_st * PI - PI / 2)
        x= -x
        z = z0
    else:
        return x0,z0

    return x,z

if __name__ == '__main__':
    t = 0.0
    x_set = []
    z_set = []
    while(t<=6.0):

        x,z = trajectory_sw(t)
        x_set.append(x)
        z_set.append(z)
        plt.plot(x_set,z_set,)
        plt.pause(0.1)

        plt.xlim((-50, 50))
        plt.ylim((-50, 50))


        plt.ioff()
        t = t+0.1

