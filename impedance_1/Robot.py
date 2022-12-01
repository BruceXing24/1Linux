# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/10/24 11:12
import pybullet as p

class robot():
    def __init__(self, robot_name):
        super().__init__()
        self.robot_name = robot_name
        self.current_vel_x = 0
        self.target_vel_x  = 0.3
        self.target_vel_z = 0.
        self.target_x = 0.
        self.target_z = 0.2835
        self.T_sw = 3.0
        self.K_vx = 0.1
        self.current_state = 0

    def get_robot_vel(self):
        vels, oris= p.getBaseVelocity(bodyUniqueId= self.robot_name)
        self.current_vel_x =vels[0]