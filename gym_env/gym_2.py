# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/26 10:00
import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data as pd
import Leg


class Dog(gym.Env):
    def __init__(self, render: bool = False):
        self.render = render

        self.action_space = spaces.Box(
            low=np.array([-np.pi, -np.pi]),
            high=np.array([np.pi, np.pi]),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=np.array([-1, -1, -1]),
            high=np.array([1,  1,  1]),
            dtype=np.float64
        )
        self.physics_client_id = p.connect(p.GUI if self.render else p.DIRECT)
        self.step_num = 0
        self.leg = Leg.LegIK()


    def step(self, action):
        self.apply_action(action)
        p.stepSimulation(physicsClientId=self.physics_client_id)
        self.step_num += 1
        state = self.get_observation()
        reward = 1
        if state[0] > np.pi/6 or self.step_num > 36000:
            done = True
        else:
            done = False
        info = {}

        return state, reward, done, info


    def reset(self):
        p.resetSimulation(physicsClientId=self.physics_client_id)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pd.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf", [0, 0, 0.7], useMaximalCoordinates=False,
                                flags=p.URDF_USE_IMPLICIT_CYLINDER)
        #  change the outlook
        p.changeVisualShape(objectUniqueId=self.robot, linkIndex=-1, rgbaColor=[1, 1, 0, 1])
        for i in range(4):
            p.changeVisualShape(objectUniqueId=self.robot, linkIndex=i * 4, rgbaColor=[1, 1, 1, 1])
            p.changeVisualShape(objectUniqueId=self.robot, linkIndex=i * 4 + 1, rgbaColor=[0.5, 0.5, 0.5, 1])
            p.changeVisualShape(objectUniqueId=self.robot, linkIndex=i * 4 + 2, rgbaColor=[0.5, 0.5, 0.5, 1])
            p.changeVisualShape(objectUniqueId=self.robot, linkIndex=i * 4 + 3, rgbaColor=[1, 0, 0, 1])

        return self.get_observation()


    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        pass

    def apply_action(self, action):
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in ")

        motor1_angle, motor2_angle = action
        motor1_angle = np.clip(motor1_angle, -np.pi, np.pi)
        motor2_angle = np.clip(motor2_angle, -np.pi, np.pi)
        self.leg.positions_control(self.robot, [motor1_angle, motor2_angle, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0])

    def get_observation(self):
        if not hasattr(self,"robot"):
            assert Exception("robot hasn't been loaded in ")
        _, baseOri = p.getBasePositionAndOrientation(self.robot,physicsClientId=self.physics_client_id)

        rpy_angle = p.getEulerFromQuaternion(baseOri)
        rpy_angle_norm = np.array(rpy_angle)/np.pi
        print(type(rpy_angle_norm))
        return rpy_angle_norm

    def close(self):
        if self.physics_client_id>=0:
            p.disconnect()
        self.physics_client_id = -1




if __name__ == '__main__':
    from stable_baselines3.common.env_checker import check_env

    env = Dog()
    check_env(env)
