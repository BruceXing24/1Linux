# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/17 13:42
import os
import pybullet as p
import pybullet_data as pd
import math
import time
import functions as f
import trajectory_generator as tg
import numpy as np



#--------------------------------------------------#
# control2 主要是验证机器人逆运动学，做了一些测试，对应位置控制参数为
# 这组参数是第一次成功跑的案例
# Force = 20
# Max_velocity = 10
# Position_Gain = 1
# Velocity_Gain = 1
# l1 = 21.1
# l2 = 19.0


#--------------------------------------------------#


PI =3.1415926
p.connect(p.GUI)
p.setGravity(0, 0, 0)
orn = p.getQuaternionFromEuler([0,0,-PI/2])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf",[0,0,0.45], useMaximalCoordinates=False,
                       flags=p.URDF_USE_IMPLICIT_CYLINDER)


num = p.getNumJoints(bodyUniqueId = robot)
print(num)
p.changeVisualShape(objectUniqueId=robot, linkIndex=-1, rgbaColor=[1, 1, 0, 1])
for i in range(4):
    p.changeVisualShape(objectUniqueId = robot, linkIndex = i*4 ,rgbaColor=[1, 1, 1, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 +1, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 2, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 3, rgbaColor=[1, 0, 0, 1])


t = 0.0
inital_count = 0
t0 = 0.0
t1 = 0.0


f.positions_control(robot, [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2])





wait_time = 0


state = 0

f.positions_control(robot, [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2])

swing_group = 'a'

A_finish_flag = 0
B_finish_flag = 0

start_swing_a = 0
start_swing_b = 0



while(1):
    contact1 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=2)  # FR
    contact2 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=6)  # FL
    contact3 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=10)  # BR
    contact4 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=14)  # BL
    print("1 , 2 ,3 , 4 =={} ,{} ,{} ,{} ".format(len(contact1), len(contact2), len(contact3), len(contact4)))

    FR_flag = len(contact1)
    FL_flag = len(contact2)
    BR_flag = len(contact3)
    BL_flag = len(contact4)
    sum = FR_flag + FL_flag + BR_flag + BL_flag
    print("to==={}".format(t0))

    if t0 >=6:
        t0 = 0
    if t1 >=6:
        t1 = 0


    if inital_count>50 and inital_count < 400:
        p.setGravity(0,0,-9.8)
        p.stepSimulation()
        time.sleep(0.01)

    if inital_count > 400:
        if state == 0:   # initial condition
            if t0 <=3.0:
                x0, z0 = tg.trajectory_sw(t0 % 6, -1.486, 28.35)
                theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                f.positions_control(robot, [0, -PI / 4, PI / 2], [0, theta1_rad, theta2_rad],
                                    [0, theta1_rad, theta2_rad],
                                    [0, -PI / 4, PI / 2])
                t0 += 0.1

                if round(t0,2) == 3.0:
                    A_finish_flag = 1
                    state = 1



        elif state ==1:
            print("initail part end")

            if sum !=4:
                wait_time+=1
                if wait_time >=15:
                    sum = 4
                    wait_time=0

            if A_finish_flag == 1 and sum == 4:
                start_swing_a = 0
                start_swing_b = 1


            elif B_finish_flag == 1 and sum == 4:
                start_swing_a = 1
                start_swing_b = 0





            if start_swing_b:
                if  t0 % 6 > 3.0  and t0 % 6<= 6.0 :
                            x0, z0 = tg.trajectory_sw(t0 % 6, -1.486, 28.35)
                            x1 ,z1 = tg.trajectory_sw(t1 % 6 ,-1.486, 28.35)

                            theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                            theta3_rad, theta4_rad = f.inverse_kinematic(x1, z1)


                            f.positions_control(robot, [0, theta3_rad, theta4_rad], [0, theta1_rad, theta2_rad],
                                                       [0, theta1_rad, theta2_rad], [0, theta3_rad, theta4_rad])

                            t0 += 0.1
                            t1 += 0.1

                            if round(t0 % 6, 2) == 6.0:
                                t0 +=0.1
                                A_finish_flag = 0
                                B_finish_flag = 1
                                start_swing_b = 0







            if start_swing_a:
                if t0 % 6 >= 0.0 and t0 % 6 <= 3.0:
                    x0, z0 = tg.trajectory_sw(t0 % 6, -1.2 + 10, 32.878)
                    x1, z1 = tg.trajectory_sw(t1 % 6, -1.2 + 10, 32.878)

                    theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                    theta3_rad, theta4_rad = f.inverse_kinematic(x1, z1)

                    f.positions_control(robot, [0, theta3_rad, theta4_rad], [0, theta1_rad, theta2_rad],
                                        [0, theta1_rad, theta2_rad], [0, theta3_rad, theta4_rad])

                    t0 += 0.1
                    t1 += 0.1

                    if t0 % 6 == 3.0:
                        t0 +=0.1
                        A_finish_flag = 1
                        B_finish_flag = 0
                        start_swing_a = 0



                # if  t0 > 3.0:
                #             x0, z0 = tg.trajectory_sw(t0 % 6, -1.2 + 10, 32.878)
                #             x1 ,z1 = tg.trajectory_sw(t1 % 6 ,-1.2+  10 , 32.878)
                #
                #             theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                #             theta3_rad, theta4_rad = f.inverse_kinematic(x1, z1)
                #
                #
                #             f.positions_control(robot, [0, theta3_rad, theta4_rad], [0, theta1_rad, theta2_rad],
                #                                        [0, theta1_rad, theta2_rad], [0, theta3_rad, theta4_rad])
                #
                #             t0 += 0.1
                #             t1 += 0.1
                #
                #     else:
                #             x0, z0 = tg.trajectory_sw(t0 % 6, -1.2 + 10, 32.878)
                #             theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                #             f.positions_control(robot, [0, -PI / 4, PI / 2], [0, theta1_rad, theta2_rad], [0, theta1_rad, theta2_rad],
                #                                 [0, -PI / 4, PI / 2])
                #             t0 += 0.1



            # elif sum ==4 and swing_group=='a':


        p.stepSimulation()
        time.sleep(0.05)


    inital_count +=1










# print(joint_state)
