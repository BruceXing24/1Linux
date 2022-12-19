# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/12/14 11:39


from woofh_leg import Leg
import pybullet as p
import numpy as np

class Robot(Leg):
    def __init__(self, robot,pybullet_client,):
        super(Leg,self).__init__()
        self.motor_angle = np.array([0] * 12)
        self._foot_id_list = [14, 19, 9, 4]
        '''                     FL         FR       BL      BR'''
        self.motor_id_list = [17, 18, 19, 12, 13, 14, 7, 8, 9, 2, 3, 4]
        self.pybullet_client = pybullet_client
        self.robot = robot
        # self.observation = self.get_observation()

    def get_base_height(self):
        posi, _ = self.pybullet_client.getBasePositionAndOrientation(self.robot)
        return np.array(posi[2])

    def get_Global_Coor(self):
        posi, _ = self.pybullet_client.getBasePositionAndOrientation(self.robot)
        return np.array(posi)

    def get_ori(self):
        _, ori = self.pybullet_client.getBasePositionAndOrientation(self.robot)
        ori =  list (self.pybullet_client.getEulerFromQuaternion(ori))
        ori[0] = ori[0]-np.pi/2  # calibration as urdf direction need to be turned pi/2
        return ori
    def get_linearV(self):
        linear_V, _ = self.pybullet_client.getBaseVelocity(self.robot)
        return linear_V

    def get_angularV(self):
        angularV, _ = self.pybullet_client.getBaseVelocity(self.robot)
        return angularV

    def get_contact(self):
        FLC = 0
        FRC = 0
        BLC = 0
        BRC = 0
        contacts = self.pybullet_client.getContactPoints()
        if len(contacts)>0:
            for contact in contacts:
                if contact[4] == 19:
                    FLC = 1
                if contact[4] == 14:
                    FRC = 1
                if contact[4] == 9:
                    BLC = 1
                if contact[4] == 4:
                    BRC = 1                     # FLC FRC
        return np.array([FLC, FRC, BLC, BRC])   # BLC BRC

    def get_reward_items(self):
        x_coor =self.get_Global_Coor()[0]
        y_coor = self.get_Global_Coor()[1]
        linerVxyz = self.get_linearV()
        angulerWxyz = self.get_angularV()
        ori = self.get_ori()
        height = self.get_base_height()
        contacts = self.get_contact()
        return np.hstack((x_coor,y_coor,linerVxyz,angulerWxyz,ori,height,contacts))

    def get_motor_angle(self):
        motor_angle = self.motor_angle
        return  motor_angle

    def get_observation(self):
        rpy = self.get_ori()
        linearXyz = self.get_linearV()
        angularXyz = self.get_angularV()
        joints_angle =self.get_motor_angle()
        contacts = self.get_contact()
        return np.hstack((rpy,linearXyz,angularXyz, joints_angle,contacts))

    def get_observation_dim(self):
        return len(self.get_observation())

    def get_observation_upper_bound(self):
        upper_bound = np.array([0.0]*self.get_observation_dim())
        upper_bound[0:3] =  2.0 * np.pi
        upper_bound[3:9] = np.inf
        upper_bound[9:21] = np.pi
        upper_bound[21:] = 1.0
        return upper_bound


if __name__ == '__main__':
    pass