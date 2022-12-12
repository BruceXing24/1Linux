import pybullet as p
import numpy as np


class Robot():
    def __init__(self,robot,physics_client_id):
        self.robot = robot
        self.motor_angle = np.array([0]*12)
        self._foot_id_list =  [14, 19, 9, 4]
        '''                     FL         FR       BL      BR'''
        self.motor_id_list = [17,18,19, 12,13,14, 7,8,9, 2,3,4]
        self.physics_client_id  = physics_client_id
        self.observation = self.get_observation()


    def get_base_height(self):
        posi, _ = p.getBasePositionAndOrientation(self.robot,physicsClientId = self.physics_client_id)
        return np.array(posi[2])

    def get_Global_Coor(self):
        posi, _ = p.getBasePositionAndOrientation(self.robot, physicsClientId = self.physics_client_id)
        return np.array(posi)


    def get_imu(self):
        _,ori = p.getBasePositionAndOrientation(self.robot, physicsClientId = self.physics_client_id )
        ori =  list (p.getEulerFromQuaternion(ori))
        ori[0] = ori[0]-np.pi/2
        linear_V, anguler_V = p.getBaseVelocity(self.robot,physicsClientId = self.physics_client_id )
        return np.array(linear_V), np.array(anguler_V) , np.array(ori)


    def get_contact(self):
        FLC = 0
        FRC = 0
        BLC = 0
        BRC = 0
        contacts = p.getContactPoints(physicsClientId = self.physics_client_id)
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


    def get_reward_items(self):
        x_coor =self.get_Global_Coor()[0]
        y_coor = self.get_Global_Coor()[1]
        linerVxyz, angulerWxyz, ori = self.get_imu()
        height = self.get_base_height()
        contacts = self.get_contact()
        return np.hstack((x_coor,y_coor,linerVxyz,angulerWxyz,ori,height,contacts))


    def get_motor_angle(self):
        # motor_angle = []
        # try:
        #     motor_angle = [p.getJointState(bodyUniqueId=self.robot, jointIndex=motor_id, physicsClientId = self.physics_client_id)[0] for motor_id in self.motor_id_list]
        # except:
        #     print("can not get angle")
        # print(len(joints))
        # FLA = np.array([ joints[0][0],joints[1][0],joints[2][0] ],dtype=np.float32)
        # FRA = np.array([ joints[3][0],joints[4][0],joints[5][0] ],dtype=np.float32)
        # BLA = np.array([ joints[6][0],joints[7][0],joints[8][0] ],dtype=np.float32)
        # BRA = np.array([ joints[9][0],joints[10][0],joints[11][0] ],dtype=np.float32)
        motor_angle = self.motor_angle

        return  motor_angle


    def get_observation(self):
        rpy = self.get_imu()[2]
        linearXyz = self.get_imu()[0]
        angularXyz = self.get_imu()[1]
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


