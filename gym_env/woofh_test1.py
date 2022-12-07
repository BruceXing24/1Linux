import pybullet as p
import pybullet_data as pd
import time
from woofh_leg import Leg
from woofh_robot import Robot
import numpy as np
from trajectory_generator import Bezier

PI = 3.1415926
id = p.connect(p.GUI)
p.setGravity(0, 0, 0)
orn = p.getQuaternionFromEuler([PI / 2, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
robot = p.loadURDF("../woofh/urdf/woofh.urdf", [0, 0, 0.3],  useMaximalCoordinates=False,
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
tg  = Bezier()
t = 0.0
leg = Leg("4legs")

while True:
    # p.resetDebugVisualizerCamera(cameraDistance=0.5,
    #                              cameraYaw =0,
    #                              cameraPitch=0 ,
    #                              cameraTargetPosition=[0,0,1]
    #                              )

    if count<100:
        time.sleep(0.02)
        p.stepSimulation()
        leg.positions_control2(  robot, [0,  -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2],
                                 [0, -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2]        )




    # inital base_mass = 0.24680316805231461
    # dt1=np.deg2rad(27)   # -27  -30
    # dt2=np.deg2rad(45)   #  90   45
    # dt3=np.deg2rad(32)   # -32  -30
    # dt4 = np.deg2rad(50) #  90   50


    # angle = count/100.0
    # leg.positions_control2(  robot, [0,  -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2],
    #                          [0, -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2]        )
    elif count>100:
     #     0.041 0.139

        time.sleep(0.5)
        p.stepSimulation()
        x,_,z = tg.curve_generator(t)
        # print(x,z)
        theta2, theta3 = leg.IK_2D(0.050,0.139)

        print("theta2=={}, theta3=={}".format(np.rad2deg(theta2), np.rad2deg(theta3)))
        leg.positions_control2(  robot, [0, 0, np.pi/2], [0, -np.pi/6, np.pi/2],
                                 [0, -np.pi/6, np.pi/2], [0, -np.pi/6, np.pi/2]        )
        t += 0.05
    # print(angle)
    # contacts = woofh.get_contact()
    # print(contacts)
    # print(count)
    count += 1


