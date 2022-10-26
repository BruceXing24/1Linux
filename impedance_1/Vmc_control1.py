# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/24 14:05


import pybullet as p
import pybullet_data as pd

import time
import functions as f
import trajectory_generator as tg
import numpy as np
import Vmc
import  Robot

PI =3.1415926
p.connect(p.GUI)
p.setGravity(0, 0, 0)
orn = p.getQuaternionFromEuler([0,0,-PI/2])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf",[0,0,0.7], useMaximalCoordinates=False,
                       flags=p.URDF_USE_IMPLICIT_CYLINDER)


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
quadruped  = Robot.robot(robot)


swing_a  = False
swing_b =  True


flag = 1
f.positions_control(robot, [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2], [0, -PI / 4, PI / 2])
while(1):

    # 检测触地
    contact1 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=2)  # FR
    contact2 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=6)  # FL
    contact3 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=10)  # BR
    contact4 = p.getContactPoints(bodyA=robot, bodyB=planeID, linkIndexA=14)  # BL
    FR_flag = len(contact1)
    FL_flag = len(contact2)
    BR_flag = len(contact3)
    BL_flag = len(contact4)
    sum = FR_flag + FL_flag + BR_flag + BL_flag
    print("1 , 2 ,3 , 4 =={} ,{} ,{} ,{} ".format(len(contact1), len(contact2), len(contact3), len(contact4)))

    # 更新速度
    quadruped.get_robot_vel()

    # 获取角度
    FR_angles = FR.get_angles()
    FL_angles = FL.get_angles()
    BL_angles = BL.get_angles()
    BR_angles = BR.get_angles()


    FR.current_x, FR.current_z = FR.forward_kinematic(FR_angles[1], FR_angles[2])
    FL.current_x, FL.current_z = FL.forward_kinematic(FL_angles[1], FL_angles[2])
    BL.current_x, BL.current_z = BL.forward_kinematic(BL_angles[1], BL_angles[2])
    BR.current_x, BR.current_z = BR.forward_kinematic(BR_angles[1], BR_angles[2])

    print("FR_x ===={},FL_x==={}".format(FR.current_x,FL.current_x))

    # 测试


    # 测试

    if inital_count>50 and inital_count < 400:
        p.setGravity(0,0,-9.80)
        p.stepSimulation()
        time.sleep(0.01)
        flag=1

    # simulation initalization

    if inital_count > 400:
        FR.time = round(FR.time, 4)
        FL.time = round(FL.time, 4)
        time_step = 0.1
        if FR_flag==1 and FL_flag==1 and BR_flag==1 and BL_flag==1:
            if (flag==1):
                quadruped.current_state = 0
            # A组设为摆动相 FR ,BL
                FR.phase = "swing"
                BL.phase = "swing"
                #每条腿时间清零
                FR.time = 0.0
                BL.time = 0.0

                # 计算摆动相初始x位置
                FR.x0 = FR.current_x
                BL.x0 = BL.current_x
                # 计算末端x位置
                FR.xf = 0.5 *quadruped.current_vel_x * quadruped.T_sw - quadruped.K_vx * \
                         (quadruped.target_vel_x - quadruped.current_vel_x)
                BL.xf =  0.5 *quadruped.current_vel_x * quadruped.T_sw - quadruped.K_vx * \
                         (quadruped.target_vel_x - quadruped.current_vel_x)

                # 计算初始z位置
                FR.z0 = FR.current_z
                BL.z0 = BL.current_z

            # b 组为支撑相
            #     FL.phase = "support"
            #     BR.phase = "support"
            #     FL.time = 0.
            #     BR.time = 0.
            #
            #     FL.target_vx = quadruped.target_vel_x
            #     BR.target_vx = quadruped.target_vel_x
            #
            #     FL.target_vz = quadruped.target_vel_z
            #     BR.target_vz = quadruped.target_vel_z
            #
            #     FL.target_z = quadruped.target_z+0.08
            #     BR.target_z = quadruped.target_z+0.08
            #
            #     flag =2
            #B 组为支撑相
            # # 给定支撑相x方向速度
            # FR.target_vx = quadruped.target_vel_x
            # # 给定支撑相z方向速度
            # FR.target_vz = quadruped.target_vel_z
            # # 给定支撑相z方向位置
            # FR.target_z = quadruped.target_z
            # if (flag == 2):
            #     quadruped.current_state = 0
            #     # B组设为摆动相 FL ,BR
            #     FL.phase = "swing"
            #     BR.phase = "swing"
            #     # 每条腿时间清零
            #     FL.time = 0.0
            #     BR.time = 0.0
            #
            #     # 计算摆动相初始x位置
            #     FL.x0 = FL.current_x
            #     BR.x0 = BR.current_x
            #     # 计算末端x位置
            #     FL.xf = 0.5 * quadruped.current_vel_x * quadruped.T_sw - quadruped.K_vx * \
            #             (quadruped.target_vel_x - quadruped.current_vel_x)
            #     BR.xf = 0.5 * quadruped.current_vel_x * quadruped.T_sw - quadruped.K_vx * \
            #             (quadruped.target_vel_x - quadruped.current_vel_x)
            #
            #     # 计算初始z位置
            #     FL.z0 = FL.current_z
            #     BR.z0 = BR.current_z
            #
            #     # b 组为支撑相 FR ,BL
            #     FR.phase = "support"
            #     BL.phase = "support"
            #     FR.time = 0.
            #     BL.time = 0.
            #
            #     FR.target_vx = quadruped.target_vel_x
            #     BL.target_vx = quadruped.target_vel_x
            #
            #     FR.target_vz = quadruped.target_vel_z
            #     BL.target_vz = quadruped.target_vel_z
            #
            #     FR.target_z = quadruped.target_z
            #     BL.target_z = quadruped.target_z
                flag = 2
            #


        #
        # x0, z0 = tg.dynamic_trajectory(h=0.05, leg_time=FR.time, T_sw=quadruped.T_sw, leg_xf=FR.xf,
        #                                leg_x0=FR.current_x, leg_z0=FR.current_z)




        #
        if flag==2 and quadruped.current_state ==0:
            FR.target_x , FR.target_z = tg.dynamic_trajectory_2(h=0.08, T_sw=quadruped.T_sw,leg=FR)
            BL.target_x , BL.target_z = tg.dynamic_trajectory_2(h=0.08, T_sw=quadruped.T_sw,leg=BL)

            print("target_X =={}".format(FR.target_x))
            # if flag==2:
            #     FL.target_x, FL.target_z = tg.dynamic_trajectory_2(h=0.08, T_sw=quadruped.T_sw, leg=FL)
            #     BR.target_x, BR.target_z = tg.dynamic_trajectory_2(h=0.08, T_sw=quadruped.T_sw, leg=BR)


            FR_fx, FR_fz = tg.force_calculate_PD(FR, time_step, quadruped.current_vel_x)
            BL_fx, BL_fz = tg.force_calculate_PD(BL, time_step, quadruped.current_vel_x)
            # FL_fx, FL_fz = tg.force_calculate_PD(FL, time_step, quadruped.current_vel_x)
            # BR_fx, BR_fz = tg.force_calculate_PD(BR, time_step, quadruped.current_vel_x)

            FR_torque1, FR_torque2 = tg.Jacobin(FR_fx, FR_fz, FR.motor_angle[1], FR.motor_angle[2])
            BL_torque1, BL_torque2 = tg.Jacobin(BL_fx, BL_fz, BL.motor_angle[1], BL.motor_angle[2])
            # FL_torque1, FL_torque2 = tg.Jacobin(FL_fx, FL_fz, FL.motor_angle[1], FL.motor_angle[2])
            # BR_torque1, BR_torque2 = tg.Jacobin(BR_fx, BR_fz, BR.motor_angle[1], BR.motor_angle[2])




            print("torque1=={},torque2=={}".format(FR_torque1, FR_torque2))
            # print("torque2=={},torque3=={}".format(FL_torque1, FL_torque2))


            FR.time += 0.1
            BL.time =+ 0.1

            f.set_torque(robot, FR_torque1, FR_torque2, 1)
            f.set_torque(robot, BL_torque1, BL_torque2, 13)
            # f.set_torque(robot, BR_torque1, BR_torque2, 9)
            # f.set_torque(robot, BL_torque1, BL_torque2, 13)


        p.stepSimulation()
        time.sleep(0.5)
        # 更新时间和位置
        FR.last_x = FR.current_x
        BL.last_x = BL.current_x
        # FL.last_x = FL.current_x
        # BR.last_x = BR.current_x

        FR.last_z = FR.current_z
        BL.last_z = BL.current_z
        # FL.last_z = FL.current_z
        # BR.last_z = BR.current_z

        FR.time = FR.time+time_step
        BL.time = BL.time+time_step
        # FL.time = FL.time+time_step
        # BR.time = BR.time+time_step


    inital_count+=1