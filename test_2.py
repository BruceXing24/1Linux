import os
import pybullet as p
import pybullet_data as pd
import math
import time

# G = 35.886154140864775 N
# mass  = 3.661852463353548
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

cheetah3_ID =  p.loadURDF("../pybullet/urdf_model/cheetah3.urdf",[0,0,0.5])

# tableUid = p.loadURDF(os.path.join(pd.getDataPath(), "table/table.urdf"), basePosition=[2, 0, 0])


num  = p.getNumJoints(cheetah3_ID)
print("num==={}".format(num))
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])
# for i in range(12):
#     joint_info = p.getJointInfo(cheetah3_ID, i)
#     print(joint_info)


count =0
while True:
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    time.sleep(0.05)
    count+=1
    if count>=100:
        for i in range(12):
            p.setJointMotorControl2(bodyIndex=cheetah3_ID,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    positionGain=1,
                                    velocityGain=1,
                                    force=500,
                                    maxVelocity=3
                                    )


        state_2=p.getJointState(bodyUniqueId=cheetah3_ID,
                        jointIndex=1
                        )
        state_5=p.getJointState(bodyUniqueId=cheetah3_ID,
                        jointIndex=4
                        )
        state_8=p.getJointState(bodyUniqueId=cheetah3_ID,
                        jointIndex=7
                        )
        state_11=p.getJointState(bodyUniqueId=cheetah3_ID,
                        jointIndex=10
                        )
        print("2,5,8,11=={},{},{},{}".format(state_2[0],state_5[0],state_8[0],state_11[0]))


        p.enableJointForceTorqueSensor(bodyUniqueId=cheetah3_ID,jointIndex=2)

        f_T = p.getJointState(bodyUniqueId=cheetah3_ID,jointIndex=2)

        force = f_T[2][0:3]

        # print("force =={}".format(force))
        body_num=p.getNumBodies()
        body_info  = p.getBodyInfo(bodyUniqueId=planeID)
        print("body_num == {}, body_info==={}".format(body_num,body_info))

        contact = p.getContactPoints(bodyA =cheetah3_ID,bodyB=planeID )
        print("contact points=={}".format(len(contact)))

        if len(contact)==4:
            print("contact1=={}".format( contact[0][9] ))
            print("contact2=={}".format(contact[1][9]))
            print("contact3=={}".format(contact[2][9]))
            print("contact4=={}".format(contact[3][9]))



        # posi,orientation=p.getBasePositionAndOrientation(bodyUniqueId=cheetah3_ID)
        # print("posi=={},orient=={}".format(posi,orientation))
        # print(state_2)



    p.stepSimulation()









