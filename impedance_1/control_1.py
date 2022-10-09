# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/9 19:43
import os
import pybullet as p
import pybullet_data as pd
import math
import time
from pybullet.impedance_1 import functions



p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
cheetah3_ID =  p.loadURDF("../pybullet/urdf_model/cheetah3.urdf",[0,0,0.5])

p.setAdditionalSearchPath(pd.getDataPath())
planeID = p.loadURDF("plane.urdf")
# objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.7])




count =0

while True:
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    time.sleep(0.05)
    count+=1
    functions.positions_control()


    p.stepSimulation()
