# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/12/17 9:56

from pybullet_utils import bullet_client
import pybullet as p
import pybullet_data as pd
import gym
from gym import spaces
import numpy as np
from woofh_robot import Robot
from trajectory_generator import Bezier
from woofh_leg import Leg
import time


class Woofh_gym(gym.Env):
    def __init__(self, render: bool = False, number_motor=12):
        self.render_flag = render
        self.woofh_leg = Leg()
        if self.render_flag:
            self._pybullet_client = bullet_client.BulletClient(connection_mode=p.GUI)
        else:
            self._pybullet_client = bullet_client.BulletClient()

        self.robot = self._pybullet_client.loadURDF("../woofh/urdf/woofh_d.urdf", [0, 0, 0.7],
                                                    useMaximalCoordinates=False,
                                                    flags=self._pybullet_client.URDF_USE_IMPLICIT_CYLINDER,
                                                    baseOrientation=self._pybullet_client.getQuaternionFromEuler(
                                                        [np.pi / 2, 0, 0]))

        self.woofh = Robot(self.robot, self._pybullet_client)

        self.action_bound = 1
        action_high = np.array([self.action_bound] * 12)
        self.action_space = spaces.Box(
            low=-action_high, high=action_high,
            dtype=np.float32)
        observation_high = self.woofh.get_observation_upper_bound()
        self.observation_space = spaces.Box(
            low=-observation_high, high=observation_high,
            dtype=np.float64)

        self.dt = 0.025  # should be related to leg control frequency
        self.forward_weightX = 0.015
        self.forward_weightY = 0.01
        self.forwardV_weight = 0.01
        self.direction_weight = -0.001
        self.shake_weight = -0.005
        self.height_weight = -0.05
        self.joint_weight = -0.001
        self.contact_weight = 0.001

        self.pre_coorX = -0.04867  # initial X
        self.pre_height = 0.1365  # initial angle

        self.gait_time = 0
        self.control_frequency = 20  # hz
        self.angleFromReferen = np.array([0] * 12)

        self.initial_count = 0
        self.tg = Bezier(step_length=0.05)

        # optimize signal
        self.opti_shoulder = np.deg2rad(3)
        self.opti_kneeAhid = np.deg2rad(15)
        self.referSignal = 1.
        self.optimSignal = 0.5

        # check everu part rewards
        self.reward_detail = np.array([0.] * 6, dtype=np.float32)
        self.reward_detail_dict = {'forwardX': 0, 'forwardY': 0, 'forwardV_reward': 0, 'direction_reward': 0,
                                   'height_reward': 0, 'contact_reward': 0}
        self.step_num = 0

    def reset(self):
        # ----------initialize pubullet env----------------
        self._pybullet_client.resetSimulation()
        self._pybullet_client.setGravity(0, 0, -9.8)
        self._pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        self.planeID = self._pybullet_client.loadURDF("plane.urdf")
        self.robot = self._pybullet_client.loadURDF("../woofh/urdf/woofh_d.urdf", [0, 0, 0.7],
                                                    useMaximalCoordinates=False,
                                                    flags=p.URDF_USE_IMPLICIT_CYLINDER,
                                                    baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
        self._pybullet_client.changeDynamics(bodyUniqueId=self.robot, linkIndex=-1, mass=1.5)

        self.woofh = Robot(self.robot, self._pybullet_client)
        #  ----------------------------------initial parameter
        self.reward_detail = np.array([0.] * 6, dtype=np.float32)
        self.step_num = 0
        self.woofh_leg.time_reset()
        self.pre_coorX = -0.04867
        self.pre_height = 0.1365
        # ----------------------------------

        while self.initial_count < 100:
            self.initial_count += 1
            self.woofh_leg.positions_control2(self.robot, [0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2],
                                              [0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2])
            self.woofh.motor_angle = np.hstack(([0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2],
                                                [0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2]))
            p.stepSimulation()
        self.initial_count = 0

        return self.get_observation()

    def get_observation(self):
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in ")
        observation = self.woofh.get_observation()
        return observation

    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        pass

    def close(self):
        if self.physics_client_id >= 0:
            self._pybullet_client.disconnect()
        self.physics_client_id = -1

    def merge_action(self, action):
        LF = [0, 0, 0]
        RF = [0, 0, 0]
        LB = [0, 0, 0]
        RB = [0, 0, 0]

        # shoulder optimize signal from -3° to 3 °
        LF[0] = action[0] * self.opti_shoulder
        RF[0] = action[3] * self.opti_shoulder
        LB[0] = action[6] * self.opti_shoulder
        RB[0] = action[9] * self.opti_shoulder

        # hip,knee optimize signal from -15° to 15 °
        LF[1:] = action[1:3] * self.opti_kneeAhid
        RF[1:] = action[4:6] * self.opti_kneeAhid
        LB[1:] = action[7:9] * self.opti_kneeAhid
        RB[1:] = action[10:] * self.opti_kneeAhid

        return np.hstack((LF, RF, LB, RB)) * self.optimSignal + self.angleFromReferen * self.referSignal

    def _reward(self, reward_items):
        x_coor = reward_items[0]
        y_coor = reward_items[1]
        linearX = reward_items[2]
        linearY, linearZ = reward_items[3:5]
        wx, wy, wz = reward_items[5:8]
        r, p, y = reward_items[8:11]
        height = reward_items[11]

        contacts = reward_items[12:]
        contact_reward = -1.
        if contacts[0] == 1 and contacts[2] == 1 and contacts[1] == 0 and contacts[3] == 0:
            contact_reward = 1.
        if contacts[0] == 0 and contacts[2] == 0 and contacts[1] == 1 and contacts[3] == 1:
            contact_reward = 1.

        # direction reward:
        forwardX_reward = self.forward_weightX * (x_coor - self.pre_coorX)
        forwardY_reward = -self.forward_weightY * np.abs(y_coor)
        forwardV_reward = self.forwardV_weight * linearX / 4
        direction_reward = self.shake_weight * self.dt * 4 * (
            np.exp((wx ** 2 + wy ** 2 + wz ** 2))) + self.shake_weight * (r ** 2 / 2 + p ** 2 / 2 + y ** 2)
        contact_reward = self.contact_weight * contact_reward
        height_reward = self.height_weight * (np.abs(height - self.pre_height))
        reward_details = np.array(
            [forwardX_reward, forwardY_reward, forwardV_reward, direction_reward, height_reward, contact_reward])
        reward = np.sum(reward_details)
        self.reward_detail += reward_details

        if self.step_num % 100 == 0:
            self.pre_coorX = x_coor
        self.pre_height = height

        return reward

    def return_reward_details(self):
        self.reward_detail_dict['forwardX'] = self.reward_detail[0]
        self.reward_detail_dict['forwardY'] = self.reward_detail[1]
        self.reward_detail_dict['forwardV_reward'] = self.reward_detail[2]
        self.reward_detail_dict['direction_reward'] = self.reward_detail[3]
        self.reward_detail_dict['height_reward'] = self.reward_detail[4]
        self.reward_detail_dict['contact_reward'] = self.reward_detail[5]

    def apply_action(self, action):
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in")

        # action_on_motor =self.merge_action(action)
        # self.leg.positions_control(self.robot, action_on_motor[0:3], action_on_motor[3:6],
        #                           action_on_motor[6:9], action_on_motor[9:12])

        if self.step_num >= 0 and self.step_num <= 20:
            random_force = np.random.uniform(-12, 12, 3)
            self._pybullet_client.applyExternalForce(objectUniqueId=self.robot, linkIndex=-1,
                                                     forceObj=[random_force[0], random_force[1], random_force[2]],
                                                     posObj=[-0.04072342, 0.00893663, 0.13637926],
                                                     flags=self._pybullet_client.WORLD_FRAME)

        x1, _, z1 = self.tg.curve_generator(self.woofh_leg.t1)
        x2, _, z2 = self.tg.curve_generator(self.woofh_leg.t2)
        theta1, theta2 = self.woofh_leg.IK_2D(x1, -z1)
        theta3, theta4 = self.woofh_leg.IK_2D(x2, -z2)

        self.angleFromReferen = np.array([0, theta1, theta2, 0, theta3, theta4, 0, theta3, theta4, 0, theta1, theta2])
        action_on_motor = self.merge_action(action)
        self.woofh_leg.positions_control2(self.robot, action_on_motor[0:3], action_on_motor[3:6],
                                          action_on_motor[6:9], action_on_motor[9:12])
        self.woofh.motor_angle = action_on_motor
        # ---------------test for free control------------------------#
        # self.woofh_leg.positions_control2( self.robot, [0, theta2 ,theta3], [0,theta4, theta5],
        #                              [0,theta4, theta5], [0, theta2 ,theta3])
        self.woofh_leg.t1 += self.dt
        self.woofh_leg.t2 += self.dt

    def step(self, action):
        self.apply_action(action)
        self._pybullet_client.stepSimulation()
        self.step_num += 1
        state = self.get_observation()
        reward_items = self.woofh.get_reward_items()
        reward = self._reward(reward_items)
        roll, pitch, yaw = self.woofh.get_ori()
        y = self.woofh.get_Global_Coor()[1]
        # condition for stop
        if self.step_num > 1000:
            done = True
        elif roll > np.deg2rad(60) or pitch > np.deg2rad(60) or yaw > np.deg2rad(60) or y > 1.:
            reward -= 10
            done = True
        else:
            done = False
        info = {}
        return state, reward, done, info

    def test_no_RL(self, model, test_round, test_speed):
        self.optimSignal = 0
        all_episode_reward = []
        height_set = []
        for i in range(test_round):
            obs = self.reset()
            episode_reward = 0
            while True:
                time.sleep(test_speed)
                action = model.predict(obs)
                obs, reward, done, _ = self.step(action[0])
                #  test for konrad
                height=self.woofh.get_reward_items()[11]
                height_set.append(height)
                episode_reward += reward
                if done:
                    break
            print("reward=={}".format(episode_reward))
            self.return_reward_details()
            print(self.reward_detail_dict)
            all_episode_reward.append(episode_reward)
        print("all_episode_reward==={}".format(all_episode_reward))
        return all_episode_reward , height_set

    def test_model(self, model, test_round,test_speed, ):
        all_episode_reward = []
        height_set = []
        for i in range(test_round):
            episode_reward = 0
            obs = self.reset()
            while True:
                time.sleep(test_speed)
                action = model.predict(obs)
                obs, reward, done, _ = self.step(action[0])
                height=self.woofh.get_reward_items()[11]
                height_set.append(height)
                episode_reward += reward
                if done:
                    break
            all_episode_reward.append(episode_reward)
            print("episode reward===={}".format(episode_reward))
            self.return_reward_details()
            print(self.reward_detail_dict)
        all_episode_reward = np.array(all_episode_reward)
        print("all_reward==={},average reward=={}".format(all_episode_reward,np.sum(all_episode_reward)/10 ))

        return all_episode_reward ,height_set

    def train_model(self, model ,train_episode, save_episode):
        save_count = 1
        all_episode_reward = []
        for i in range(train_episode):
            episode_reward = 0
            obs = self.reset()
            while True:
                action = model.predict(obs)
                obs, reward, done, _ = self.step(action[0])
                episode_reward += reward
                if done:
                    break

            all_episode_reward.append(episode_reward)
            # print('episode_reward==={}'.format(episode_reward))
            # print("train=={}".format(i))

            if i > 1 and i % save_episode == 0:
                model.save('result/train_result' + str(save_count))
                save_count += 1
        file = open('result/reward.txt', 'w')
        file.write(str(all_episode_reward))


if __name__ == '__main__':
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3 import PPO
    import matplotlib.pyplot as plt


    env = Woofh_gym(render=True)
    # model = PPO(policy="MlpPolicy", env=env, batch_size=256, verbose=1, tensorboard_log="./result/")
    # model = PPO(policy="MlpPolicy", env=env, batch_size=256, verbose=1)
    # _, height_set=env.test_no_RL(model,3,0)







    # plt.plot(height_set)
    # plt.show()

    #
    # t1 = time.time()
    # model.learn(1500000)
    # model.save('result/train_result_15m')
    # t2 = time.time()
    # print(t2-t1)

    loaded_model=PPO.load('result/train_result_15m')
    _, height_set = env.test_model(loaded_model,3,0.01)






    # check_env(env)
