import pybullet as p
import pybullet_data as pd
import time
from woofh_leg import Leg
from woofh_robot import Robot


PI = 3.1415926
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
orn = p.getQuaternionFromEuler([PI / 2, 0, 0])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
robot = p.loadURDF("../woofh/urdf/woofh.urdf", [0, 0, 0.7], useMaximalCoordinates=False,
                   flags=p.URDF_USE_IMPLICIT_CYLINDER, baseOrientation=orn)

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
leg = Leg("4legs")
woofh = Robot(robot)
while True:
    time.sleep(0.1)
    p.stepSimulation()

    # posi , ori = robot.get_Global_Coor()
    # print("global_position is{},ori is {}".format(posi,ori))
    linear_V, anguler_V = woofh.get_imu()
    print("linear_V=={},angular_V=={}".format(linear_V,anguler_V))




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
