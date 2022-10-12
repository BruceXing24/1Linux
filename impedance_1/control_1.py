import os
import pybullet as p
import pybullet_data as pd
import math
import time
import functions

PI =3.1415926 

p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

cheetah3_ID =  p.loadURDF("/home/roslab/Remote/1Linux/urdf_model/cheetah3.urdf",[0,0,0.5])
p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])



count =0

while True:
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    time.sleep(0.05)


    if count >=100:
        functions.positions_control(cheetah3_ID,[0,PI/4,0],[0,PI/4,0],[0,PI/4,0],[0,PI/4,0])

    
    p.stepSimulation()
    pos_1=p.getJointStates(cheetah3_ID,[1,4,7,10])[0][0]
    pos_2=p.getJointStates(cheetah3_ID,[1,4,7,10])[1][0]
    pos_3=p.getJointStates(cheetah3_ID,[1,4,7,10])[2][0]
    pos_4=p.getJointStates(cheetah3_ID,[1,4,7,10])[3][0]


    print("pos1 = {:.3}, posi_2=={:.3}, pos3=={:.3}, pos4=={:.3}".format(pos_1,pos_2,pos_3,pos_4))


    count+=1
