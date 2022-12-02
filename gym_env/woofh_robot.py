import pybullet as p
import numpy as np


class Robot():
    def __init__(self,robot):
        self.robot = robot
        self._foot_id_list =  [14, 19, 9, 4]
        self.observation = self.get_observation()

    def get_base_height(self):
        posi, _ = p.getBasePositionAndOrientation(self.robot)
        return np.array(posi[2])

    def get_Global_Coor(self):
        posi, _ = p.getBasePositionAndOrientation(self.robot)
        return np.array(posi)


    def get_imu(self):
        _,ori = p.getBasePositionAndOrientation(self.robot)
        ori =  p.getEulerFromQuaternion(ori)
        linear_V, anguler_V = p.getBaseVelocity(self.robot)
        return np.array(linear_V), np.array(anguler_V) , np.array(ori)


    def get_contact(self):
        FLC = 0
        FRC = 0
        BLC = 0
        BRC = 0
        contacts = p.getContactPoints()
        if len(contacts)>0:
            for contact in contacts:
                if contact[4] == 19:
                    FLC = 1
                if contact[4] == 14:
                    FRC = 1
                if contact[4] == 9:
                    BLC = 1
                if contact[4] == 4:
                    BRC = 1
        return np.array([FLC, FRC, BLC, BRC])

    def get_motor_angle(self):
        joints = p.getJointStates(self.robot)
        FLA = np.array([ joints[17][0],joints[18][0],joints[19][0] ],dtype=np.float32)
        FRA = np.array([ joints[12][0],joints[13][0],joints[14][0] ],dtype=np.float32)
        BLA = np.array([ joints[7][0],joints[8][0],joints[9][0] ],dtype=np.float32)
        BRA = np.array([ joints[2][0],joints[3][0],joints[4][0] ],dtype=np.float32)
        return  np.hstack((FLA,FRA,BLA,BRA))


    def get_observation(self):
        rpy = self.get_imu()[2]
        linearXyz = self.get_imu()[0]
        angularXyz = self.get_imu()[1]
        joints_angle =self.get_motor_angle()
        contacts = self.get_contact()
        return np.hstack((rpy,linearXyz,angularXyz, joints_angle,contacts))

