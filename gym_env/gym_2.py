# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/26 10:00
import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data as pd
from woofh_leg import Leg
from woofh_robot import Robot
import time
from trajectory_generator import Bezier


class Dog(gym.Env):
    def __init__(self, render: bool = False , number_motor = 12):
        self.render = render
        # for action space
        self.action_dim = number_motor
        self.action_bound = 1
        action_high = np.array([self.action_bound]*12)
        self.action_space = spaces.Box(
            low=-action_high, high=action_high,
            dtype=np.float32
        )
        self.physics_client_id = p.connect(p.GUI if self.render else p.DIRECT)
        self.step_num = 0
        self.leg = Leg("4leg")
        self.robot = p.loadURDF("../woofh/urdf/woofh.urdf", [0, 0, 0.7], useMaximalCoordinates=False,
                                flags=p.URDF_USE_IMPLICIT_CYLINDER,baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
        self.woofh = Robot(self.robot, physics_client_id=self.physics_client_id)


        # for observation_space
        observation_high = self.woofh.get_observation_upper_bound()
        self.observation_space = spaces.Box(
            low  = -observation_high,  high = observation_high,
            dtype=np.float64
        )

        self.dt = 0.02
        self.forward_weight = 0.01
        self.direction_weight = -0.001
        self.shake_weight = -0.01
        self.height_weight = -0.01
        self.joint_weight = -0.001

        self.pre_coorX= -0.04867
        self.pre_height = 0.140

        # params for gait controller
        self.gait_time = 0
        self.control_frequency = 20   # hz
        self.angleFromReferen = np.array([0]*12)



    def step(self, action):

        # tg_trajectory

        # self.angleFromReferen = tg.curve(self.gait_time)
        #


        self.apply_action(action)
        p.stepSimulation(physicsClientId=self.physics_client_id)
        self.step_num += 1
        state = self.get_observation()

        reward_items  = self.woofh.get_reward_items()

        reward = self._reward(reward_items)
        print("reward==={}".format(reward))

        self.pre_coorX = reward_items[0]
        self.pre_height = reward_items[10]

        # condition for stop
        if self.step_num > 36000:
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
        self.robot = p.loadURDF("../woofh/urdf/woofh.urdf", [0, 0, 0.7], useMaximalCoordinates=False,
                                flags=p.URDF_USE_IMPLICIT_CYLINDER,baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
        #  change the outlook
        self.woofh = Robot(self.robot, physics_client_id=self.physics_client_id)
        return self.get_observation()


    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        pass

    def _reward(self,reward_items):
        x_coor = reward_items[0]
        linearX = reward_items[1]
        linearY,linearZ  = reward_items[2:4]
        wx ,wy ,wz = reward_items[4:7]
        r , p , y  = reward_items[7:10]
        height = reward_items [10]

        reward = self.forward_weight *   (x_coor-self.pre_coorX) + \
                 self.forward_weight *    np.exp(-4*linearX) + \
                 self.direction_weight * (linearY**2+linearZ**2)+ \
                 self.shake_weight*      ( np.exp( -1/(wx**2+wy**2+wz**2) ))+ \
                 self.shake_weight*      ( r**2/2+p**2/2+y**2)+ \
                 self.height_weight *    ( height-self.pre_height)

        return reward

    def apply_action(self, action):
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in")


        # action_on_motor =self.merge_action(action)
        # self.leg.positions_control(self.robot, action_on_motor[0:3], action_on_motor[3:6],
        #                           action_on_motor[6:9], action_on_motor[9:12])



        self.leg.positions_control(self.robot, action[0][0:3],action[0][3:6],action[0][6:9],action[0][9:12])



    def get_observation(self):
        if not hasattr(self,"robot"):
            assert Exception("robot hasn't been loaded in ")
        observation = self.woofh.get_observation()
        return observation





    def close(self):
        if self.physics_client_id>=0:
            p.disconnect()
        self.physics_client_id = -1

    def merge_action(self,action):
        action_on_motor = np.array([0]*12)
        LF = [0, 0, 0]
        RF = [0, 0, 0]
        LB = [0, 0, 0]
        RB = [0, 0, 0]

        # shoulder optimize signal from -5° to 5 °
        LF[0] = action[0][0] * np.deg2rad(5)
        RF[0] = action[0][3] * np.deg2rad(5)
        LB[0] = action[0][6] * np.deg2rad(5)
        RB[0] = action[0][9] * np.deg2rad(5)

        # hip,knee optimize signal from -15° to 15 °
        LF[1:] = action[0][1:3] * np.deg2rad(15)
        RF[1:] = action[0][4:6] * np.deg2rad(15)
        LB[1:] = action[0][7:9] * np.deg2rad(15)
        RB[1:] = action[0][10:] * np.deg2rad(15)

        return  np.hstack((LF, RF , LB, RB))+self.angleFromReferen







if __name__ == '__main__':
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3 import PPO2
    env = Dog(render=True)
    obs = env.reset()
    p.setRealTimeSimulation(1)
    model = PPO2(policy = "MlpPolicy",env = env



                 )


    while True:
        time.sleep(0.5)
        action = np.random.rand(1,12)
        state ,reward ,done, _ = env.step(action)
        print("state==={}".format(state))

        if done:
            break

    # check_env(env)
