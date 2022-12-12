import pybullet as p
import pybullet_data as pd
import time
from woofh_leg import Leg
from woofh_robot import Robot
import numpy as np
from trajectory_generator import Bezier

dt = 0.02
forward_weight = 0.01
direction_weight = -0.001
shake_weight = -0.01
height_weight = -0.01
joint_weight = -0.001
contact_weight = 0.001

PI = 3.1415926
id = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
orn = p.getQuaternionFromEuler([PI / 2, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
robot = p.loadURDF("../woofh/urdf/woofh_d.urdf", [0, 0, 0.2],  useMaximalCoordinates=False,
                   flags=p.URDF_USE_IMPLICIT_CYLINDER,       baseOrientation=orn         )


#  change the outlook
p.changeVisualShape(objectUniqueId=robot, linkIndex=-1, rgbaColor=[1, 1, 0, 1])
for i in range(4):
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 5, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 5 + 1, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 5 + 2, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 5 + 3, rgbaColor=[1, 0, 0, 1])
    p.changeVisualShape(objectUniqueId=robot, linkIndex=i * 5 + 4, rgbaColor=[1, 1, 0.5, 1])
p.changeVisualShape(objectUniqueId=robot, linkIndex=20, rgbaColor=[0.5, 0.5, 0.5, 1])
p.changeVisualShape(objectUniqueId=robot, linkIndex=21, rgbaColor=[0.5, 0.5, 0.5, 1])
joint_info = p.getNumJoints(bodyUniqueId=robot)
print("joint_info==={}".format(joint_info))

count = 0
woofh = Robot(robot,physics_client_id = id)
tg  = Bezier(step_length=0.05)
t1 = 0.0
t2 = -.5
leg = Leg("4legs")

p.changeDynamics(bodyUniqueId = robot,linkIndex = -1, mass= 1.5)

while True:
    # p.resetDebugVisualizerCamera(cameraDistance=0.5,
    #                              cameraYaw =0,
    #                              cameraPitch=0 ,
    #                              cameraTargetPosition=[0,0,1]
    #                              )
    angle = count/2000.
    episode_reward = 0
    reward = 0

    if count<100:
        time.sleep(0.02)
        p.stepSimulation()
        #                                                               calia -0.
        leg.positions_control2(  robot, [0, 0-np.pi/4, np.pi/2],
                             [0,  -np.pi/4  , np.pi/2    ]    ,
                             [0, -np.pi/4  , np.pi/2       ]   ,
                             [0,-np.pi/4  , np.pi/2     ]    )


        woofh.motor_angle = np.hstack( ( np.array([0, 0-np.pi/4, np.pi/2]), np.array([0, 0-np.pi/4, np.pi/2]),
                                         np.array([0, 0-np.pi/4, np.pi/2]), np.array([0, 0-np.pi/4, np.pi/2])   ))

        print("-------------------------")
        print(woofh.get_motor_angle())
        # contacts = woofh.get_contact()
        # print(contacts)
        # if contacts[0]==1 and contacts[1]==1 and contacts[2]==1 and contacts[3]==1:
        #     print("caliabration =={} ".format(angle))
        # _,_,rpy = woofh.get_imu()




    # inital base_mass = 0.24680316805231461
    # dt1=np.deg2rad(27)   # -27  -30
    # dt2=np.deg2rad(45)   #  90   45
    # dt3=np.deg2rad(32)   # -32  -30
    # dt4 = np.deg2rad(50) #  90   50


    # angle = count/100.0
    # leg.positions_control2(  robot, [0,  -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2],
    #                          [0, -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2]        )


    elif count>300 and count<1500:
        if count >= 300 and count <= 320:
            random_force  = np.random.uniform(-12,12,3)
            p.applyExternalForce(objectUniqueId=robot, linkIndex=-1,
                             forceObj=[random_force[0], random_force[1],random_force[2]],
                             posObj=[-0.04072342, 0.00893663, 0.13637926],
                             flags=p.WORLD_FRAME
                             )
     #     0.041 0.139
     #    if count >= 600 and count <= 605:
     #        random_force  = np.random.uniform(-13,13,3)
     #        p.applyExternalForce(objectUniqueId=robot, linkIndex=-1,
     #                         forceObj=[random_force[0], random_force[1],random_force[2]],
     #                         posObj=[-0.04072342, 0.00893663, 0.13637926],
     #                         flags=p.WORLD_FRAME
     #                         )
     #
        time.sleep(0.01)
        p.stepSimulation()
        x1,_,z1 = tg.curve_generator(t1)
        x2, _, z2 = tg.curve_generator(t2)

        theta2, theta3 = leg.IK_2D(x1 ,-z1)
        theta4, theta5 = leg.IK_2D(x2 ,-z2)


        # print("theta2=={}, theta3=={}".format(np.rad2deg(theta2), np.rad2deg(theta3)))
        # leg.positions_control2(robot, [0, 0 - np.pi / 4, np.pi / 2],
        #                    [0, -np.pi / 4, np.pi / 2],
        #                    [0, -np.pi / 4, np.pi / 2],
        #                    [0, -np.pi / 4, np.pi / 2])
        #
        # leg.positions_control2(  robot, [0, theta2 ,theta3], [0,theta4, theta5],
        #                              [0,theta4, theta5], [0, theta2 ,theta3]     )
        t1 += 0.025
        t2 += 0.025
        posi =   woofh.get_Global_Coor()
        obs = woofh.get_observation()
        print(posi)


    elif count>1500:

        print(reward)
        p.disconnect(id)





    # print(angle)

    # print(count)
    count += 1
    print(count)

