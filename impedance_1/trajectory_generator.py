# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/4 15:49
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import Vmc

T_sw = 3.0
T_st = 3.0
s = 0.03
h=  0.05
PI = np.pi
l1 = 0.211
l2 = 0.19

def trajectory_sw(t:float , x0, z0):
    if t>=0 and t<=T_sw:
        x = x0 + s* np.sin(t/T_sw* PI- PI/2)
        z = z0 - h* np.sin(PI - t/T_sw * PI)

    elif t>T_sw <=T_sw+T_st:
        t = t-T_sw
        x = -x0 + s * np.sin(t / T_st * PI - PI / 2)
        x = -x
        z = z0
    else:
        return x0,z0
    return x,z


def trajectory_sw_2(t:float , x0, z0):
    if t>=0 and t<=T_sw:
        x = x0 + s* np.sin(t/T_sw* PI- PI/2)
        z = z0 - h* np.sin(PI - t/T_sw * PI)

    elif t>T_sw:
        x = x0 + s* np.sin( PI- PI/2)
        z = z0 - h * np.sin(PI - PI)

    else:
        return x0 + s* np.sin( PI- PI/2),z0
    return x,z



def dynamic_trajectory(h , leg_time, T_sw ,leg_xf,leg_x0,leg_z0 ):
    if (leg_time< T_sw):
        sigma =  2* PI * leg_time/ T_sw
        x = (leg_xf-leg_x0) *  ( (sigma-np.sin(sigma) )/(2*PI)  ) +leg_x0
        z = h * (1-np.cos(sigma))/2 + leg_z0
    else:
        x = leg_xf
        z = leg_z0
    return x,z



def dynamic_trajectory_2(h , T_sw, leg:Vmc.Leg ):
    if (leg.time< T_sw):
        sigma =  2* PI * leg.time/ T_sw
        x = (leg.xf-leg.x0) *  ( (sigma-np.sin(sigma) )/(2*PI)  ) +leg.x0
        z = h * (1-np.cos(sigma))/2 + leg.z0
    else:
        x = leg.xf
        z = leg.z0
    return x,z



# xf = -0.5* Robot.current_vx * Robot.T_sw - Robot.K_vx（robot.Target_vx - robot.current_vx）

def Jacobin(virtual_fx , virtual_fz, theta1 ,theta2):
    # Torque(1) = (obj.hl * cos(leg.th1 + leg.th2) + obj.hu * cos(leg.th1)) * Force.fx + (
    #             - obj.hl * sin(leg.th1 + leg.th2) - obj.hu * sin(leg.th1)) * Force.fz;
    # Torque(2) = obj.hl * cos(leg.th1 + leg.th2) * Force.fx + (-obj.hl * sin(leg.th1 + leg.th2)) * Force.fz;

    torque_1 = l2 * np.cos(theta1+ theta2) + l1 *np.cos(theta1)*virtual_fx + \
               (-l2 *np.sin(theta1+theta2)-l1*np.sin(theta1)*virtual_fz     )

    torque_2 = l2 *np.cos(theta1+theta2)*virtual_fx + (-l2*np.sin(theta1+theta2))*virtual_fz

    return torque_1, torque_2


def force_calculate_PD(leg:Vmc.Leg, time_step,robot_current_vx):
    Kp = 2000
    Kd = 100

    if leg.phase == "swing":
        vx = (leg.current_x - leg.last_x) / time_step
        vz = (leg.current_z - leg.last_z) / time_step
        virtual_fx = Kp * (leg.target_x - leg.current_x) + Kd*(leg.target_vx - vx)
        virtual_fz = Kp * (leg.target_z - leg.current_z) + Kd * (leg.target_vz - vz)
        return virtual_fx, virtual_fz



    elif leg.phase == "support":
        Kp2 = 500
        Kd2_x = 100
        Kd2_z = 90

        vz= (leg.current_z - leg.last_z)/time_step
        vx = -robot_current_vx

        virtual_fx = Kd2_x* (leg.target_vx - vx)
        virtual_fz = Kp2 * (leg.target_z - leg.current_z) + Kd2_z*(leg.target_vz-vz)

        return virtual_fx, virtual_fz





if __name__ == '__main__':
    t = 0.0
    x_set = []
    z_set = []
    x1_set = []
    z1_set = []

    while(t<=6.0):

        # xf = -0.5* Robot.current_vx * Robot.T_sw - Robot.K_vx（robot.Target_vx - robot.current_vx）
        T = 3
        current_vx = 0
        Target_vx = 0.3
        K_vx = 15
        xf = -0.5 * current_vx * T - 0.1 * (Target_vx - current_vx)
        print("xf==={}".format(xf))
        print(xf)
        # x , z = dynamic_trajectory(h=-0.05, leg_time=t , T_sw=3, leg_xf=xf, leg_x0 = -0.01486,leg_z0= 0.02835)
        # print(x, z)

        x1,z1 = trajectory_sw(t,-1.486/100 , 28.35/100)
        print(x1,z1)

        print("------------")
        # x_set.append(x)
        # z_set.append(z)
        #
        x1_set.append(x1)
        z1_set.append(z1)
        # plt.plot(x_set,z_set,)
        plt.plot(x1_set,z1_set)
        plt.pause(0.1)





        plt.xlim((-.3, .3))
        plt.ylim((-.3, .3))


        plt.ioff()
        t = t+0.1

