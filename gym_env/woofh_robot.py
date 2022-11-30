import pybullet as p



class Robot():
    def __init__(self,robot):
        self.robot = robot

    def get_Global_Coor(self):
        posi, ori = p.getBasePositionAndOrientation(self.robot)
        ori  = p.getEulerFromQuaternion(ori)
        return posi , ori

    def get_imu(self):
        _,ori = p.getBasePositionAndOrientation(self.robot)
        linear_V, anguler_V = p.getBaseVelocity(self.robot)

        return linear_V, anguler_V




