import pybullet as p
import pybullet_data as pd
import time
from woofh_leg import Leg
from woofh_robot import Robot
import numpy as np

PI = 3.1415926
id = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
orn = p.getQuaternionFromEuler([PI / 2, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
robot = p.loadURDF("../woofh/urdf/woofh.urdf", [0, 0, 0.3], useMaximalCoordinates=False,
                   flags=p.URDF_USE_IMPLICIT_CYLINDER,baseOrientation=orn)


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

while True:

    # p.resetDebugVisualizerCamera(cameraDistance=0.5,
    #                              cameraYaw =0,
    #                              cameraPitch=0 ,
    #                              cameraTargetPosition=[0,0,1]
    #                              )

    time.sleep(0.1)
    p.stepSimulation()
    leg = Leg("4legs")

    # inital base_mass = 0.24680316805231461
    dt1=np.deg2rad(27)
    dt2=np.deg2rad(45)

    dt3=np.deg2rad(32)
    dt4 = np.deg2rad(50)
    leg.positions_control(robot, [0,-dt1,dt2], [0,-dt3,dt4], [0,-dt1,dt2], [0,-dt1,dt2])
    contacts = woofh.get_contact()

    print(contacts)
    posi, ori = p.getBasePositionAndOrientation(robot)
    print("posi==={}".format(posi))







    # motor_angle = woofh.get_motor_angle()
    observation = woofh.get_observation()
    # print(motor_angle)
    print("rpy  ==={:5f},{:5f},{:5f}".format(observation[0],observation[1],observation[2]))
    print("vx,vy,vz=== {:5f},{:5f},{:5f}".format(observation[3],observation[4],observation[5]))
    print("wx,wy,wz=== {:5f},{:5f},{:5f}".format(observation[6], observation[7], observation[8]))


    # test for all shoulders
    # leg.positions_control(robot, [PI/4, 0, 0],[PI/4, 0, 0],[PI/4, 0, 0],[PI/4, 0, 0] )
    # # test forall hips
    # leg.positions_control(robot, [ 0, PI / 4, 0], [ 0, PI / 4, 0], [ 0, PI / 4, 0], [ 0, PI / 4, 0])
    #
    # leg.positions_control(robot, [0,  0, PI / 4], [0,  0, PI / 4], [0,  0, PI / 4], [0,  0, PI / 4])

    # for i in range(4):
    #     p.setJointMotorControl2(bodyIndex=robot,
    #                             jointIndex=5*i+4,
    #                             controlMode=p.POSITION_CONTROL,
    #                             targetPosition=PI/4,
    #                             positionGain=1,
    #                             velocityGain=1,
    #                             force=10,
    #                             maxVelocity=5)

    # p.setJointMotorControl2(bodyIndex=robot,
    #                         jointIndex=9,
    #                         controlMode=p.POSITION_CONTROL,
    #                         targetPosition=PI/4,
    #                         positionGain=1,
    #                         velocityGain=1,
    #                         force=10,
    #                         maxVelocity=5)

    count += 1
