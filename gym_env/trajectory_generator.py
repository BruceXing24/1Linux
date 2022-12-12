# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/29 15:52
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Bezier:
    def __init__(self,step_length = 0.1):
        self.Tswing = 0.5    # unit = s
        self.Tsupport = 0.5  # unit = s
        self.step_length  = step_length
        # theta1 =-30 theta2=90
        # 0.041
        self.initial_x = 0.0
        self.initial_y = 0.139
        self.initial_z = 0.047

        # transfer to world coordinate
        self.P0 = np.array([0 + self.initial_x,             self.initial_z, 0  - self.initial_y ] )
        self.P1 = np.array([self.step_length/10     + self.initial_x,          self.initial_z, 0.1- self.initial_y ] )
        self.P2 = np.array([self.step_length * 9/10 + self.initial_x,          self.initial_z, 0.1- self.initial_y ] )
        self.P3 = np.array([self.step_length+self.initial_x,self.initial_z, 0  - self.initial_y ] )


    def curve_generator(self,t):
        t = t % 1.0
        point = [0.0,  0 , -0.139 ]
        if t<0:
            point = [0.0,  0 , -0.139 ]


        if t>=0 and t <= self.Tswing:
            t1 =t *2
            point = self.P0*(1-t1)**3 +\
                    3*self.P1*t1* ((1-t1)**2) + \
                    3*self.P2*(t1**2)*(1-t1)+\
                    self.P3*(t1**3)

        if t> self.Tswing and t <=self.Tswing+self.Tsupport:
            point = [   -0.1 *t +0.1 ,  0  , -0.139]

        return point



if __name__ == '__main__':
    tg = Bezier()
    t = 0
    x_set = []
    y_set = []
    z_set = []
    fig = plt.figure()
    ax1 = plt.axes(projection = '3d')

    while(True):
        point=tg.curve_generator(t)
        x_set.append(point[0])
        z_set.append(point[1])
        y_set.append(point[2])
        ax1.plot3D(x_set,y_set,z_set,'red')
        ax1.set_xlim(-.1, 0.2)
        ax1.set_ylim(-.1, 0.2)
        ax1.set_zlim(-.1, 0.2)
        plt.pause(0.1)
        plt.ioff()
        t = t + 0.1
        if t>1.0:
            print(x_set)
            print(y_set)
            print(z_set)


    # 二维测试
    # while(True):
    #     point=tg.curve_generator(t)
    #
    #     x_set.append(point[0])
    #     z_set.append(point[1])
    #     y_set.append(point[2])
    #     plt.plot(x_set,z_set,y_set)
    #     plt.pause(0.1)
    #
    #     plt.xlim((-.1, .2 , .2))
    #     plt.ylim((-.1, .2 , .2))
    #     plt.ioff()
    #
    #     t = t + 0.01


