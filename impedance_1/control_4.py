# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/17 13:42

import pybullet as p
import pybullet_data as pd

import time
import functions as f
import trajectory_generator as tg
import numpy as np
import Vmc




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
robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf",[0,0,0.40], useMaximalCoordinates=False,
                       flags=p.URDF_USE_IMPLICIT_CYLINDER)

BoxId = p.loadURDF("r2d2.urdf", [1,1,1])

p.changeVisualShape(objectUniqueId=robot, linkIndex=-1, rgbaColor=[1, 1, 0, 1])
for i in range(4):
    p.changeVisualShape(objectUniqueId = robot, linkIndex = i*4 ,rgbaColor=[1, 1, 1, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 +1, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 2, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 4 + 3, rgbaColor=[1, 0, 0, 1])


inital_count = 0
f.positions_control(robot, [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2])

FR  = Vmc.Leg(robot,[0,1,2],"FR")
FL  = Vmc.Leg(robot,[4,5,6],"FL")
BR  = Vmc.Leg(robot,[8,9,10],"BR")
BL  = Vmc.Leg(robot,[12,13,14],"BL")

swing_a_finish = False
swing_b_finish = False

start_swing_b =False
start_swing_a =False

FR.time = 0.0
FL.time = 0.0

inital_state = 1


while(1):
    contact1 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=2)  # FR
    contact2 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=6)  # FL
    contact3 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=10)  # BR
    contact4 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=14)  # BL
    try:
        print("force 1  ".format(contact1[9]))
    except:
        print("no touching")

    print("1 , 2 ,3 , 4 =={} ,{} ,{} ,{} ".format(len(contact1), len(contact2), len(contact3), len(contact4)))
    FR_flag = len(contact1)
    FL_flag = len(contact2)
    BR_flag = len(contact3)
    BL_flag = len(contact4)
    sum = FR_flag + FL_flag + BR_flag + BL_flag

    robot_velocity=p.getBaseVelocity(bodyUniqueId = robot)
    print("vel = {}".format(robot_velocity))

    # inintiate robot
    if inital_count>50 and inital_count < 400:
        p.setGravity(0,0,-9.8)
        p.stepSimulation()
        time.sleep(0.01)


    # simulation initalization

    if inital_count > 400:
        FR.time = round(FR.time, 4)
        FL.time = round(FL.time, 4)
        print("FR time ==={}, FL time =={}, sum=={}".format(FR.time,FL.time, sum ))

        if inital_state == 1:
            if FR.time <=3.0:
                x0, z0 = tg.trajectory_sw(FR.time % 6.0, -1.486, 28.35)
                theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                FR.postion_control([0,theta1_rad,theta2_rad])
                BL.postion_control([0, theta1_rad, theta2_rad])
                FR.time+=0.1
            else:
                inital_state = 0
                swing_a_finish = True
                swing_b_finish = False


        if swing_a_finish ==True and sum ==4:
            start_swing_b = True
            start_swing_a = False

        elif swing_b_finish ==True and sum ==4:
            start_swing_b = False
            start_swing_a = True


        if inital_state ==0:
            if start_swing_b ==True:

                print("swing b ....")
                if FL.time <= 3.0:
                    x1 ,z1 = tg.trajectory_sw(FL.time % 6.0, -1.486, 28.35)
                    theta3_rad, theta4_rad = f.inverse_kinematic(x1, z1)

                    FL.postion_control([0, theta3_rad, theta4_rad])
                    BR.postion_control([0, theta3_rad, theta4_rad])

                    x0, z0 = tg.trajectory_sw(FR.time % 6.0, -1.486, 28.35)
                    theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                    FR.postion_control([0, theta1_rad, theta2_rad])
                    BL.postion_control([0, theta1_rad, theta2_rad])


                    FR.time += 0.1
                    FL.time += 0.1

                    FR.time = f.initalize_timer(FR.time, 6.0)

                    swing_a_finish = False
                    swing_b_finish = False

                else:
                    swing_a_finish = False
                    swing_b_finish = True



            elif start_swing_a ==True:
                print("swing a ....")
                if FR.time <= 3.0:
                    x0, z0 = tg.trajectory_sw(FR.time % 6.0, -1.486, 28.35)
                    theta1_rad, theta2_rad = f.inverse_kinematic(x0, z0)
                    FR.postion_control([0, theta1_rad, theta2_rad])
                    BL.postion_control([0, theta1_rad, theta2_rad])



                    x1 ,z1 = tg.trajectory_sw(FL.time % 6.0, -1.486, 28.35)
                    theta3_rad, theta4_rad = f.inverse_kinematic(x1, z1)
                    FL.postion_control([0, theta3_rad, theta4_rad])
                    BR.postion_control([0, theta3_rad, theta4_rad])


                    FR.time += 0.1
                    FL.time += 0.1

                    FL.time = f.initalize_timer(FL.time, 6.0)

                    swing_a_finish = False
                    swing_b_finish = False
                else:
                    swing_a_finish = True
                    swing_b_finish = False

        p.stepSimulation()
        time.sleep(0.02)




            # world_position  =  p.getLinkState(bodyUniqueId=robot,linkIndex=3
    inital_count +=1










# print(joint_state)
