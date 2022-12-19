# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/26 10:00
import torch
import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data as pd
from woofh_leg import Leg
from woofh_robot import Robot
import time
from trajectory_generator import Bezier

np.set_printoptions(suppress=True)

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
        # self.action_space = spaces.Box(
        #     low= np.array([-1,-1,-1, -1,-1,-1, -1,-1,-1, -1,-1,-1]),
        #     high=np.array([ 1, 1, 1,  1, 1, 1,  1, 1, 1,  1, 1, 1]),
        #     dtype=np.float32
        # )

        self.physics_client_id = p.connect(p.GUI if self.render else p.DIRECT)
        self.step_num = 0
        self.leg = Leg("4leg")
        self.robot = p.loadURDF("../woofh/urdf/woofh_d.urdf", [0, 0, 0.7], useMaximalCoordinates=False,
                                flags=p.URDF_USE_IMPLICIT_CYLINDER,baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
        self.woofh = Robot(self.robot, physics_client_id=self.physics_client_id)


        # for observation_space
        observation_high = self.woofh.get_observation_upper_bound()
        self.observation_space = spaces.Box(
            low  = -observation_high,  high = observation_high,
            dtype=np.float64
        )

        self.dt = 0.025
        self.forward_weightX = 0.5 # 0.05
        self.forward_weightY = 0.02
        self.forwardV_weight = 0.03
        self.direction_weight = -0.001
        self.shake_weight = -0.003
        self.height_weight = -0.005
        self.joint_weight = -0.001
        self.contact_weight  = 0.001

        self.pre_coorX= -0.04867
        self.pre_height = 0.1365

        # params for gait controller
        self.gait_time = 0
        self.control_frequency = 20   # hz
        self.angleFromReferen = np.array([0]*12)

        self.initial_count = 0
        self.tg = Bezier(step_length=0.05)

        # optimize signal
        self.opti_shoulder = np.deg2rad(5)
        self.opti_kneeAhid = np.deg2rad(15)
        self.referSignal  = 1.
        self.optimSignal = 0.5


        # check everu part rewards
        self.reward_detail = np.array([0.]*6, dtype=np.float32)



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
        roll, pitch ,yaw = self.woofh.get_imu()[2]

        # condition for stop
        if self.step_num > 1000 or roll > np.deg2rad(35) or pitch >np.deg2rad(35) or yaw >np.deg2rad(35):
            done = True
        else:
            done = False
        info = {}

        return state, reward, done, info


    def reset(self):
        p.resetSimulation(physicsClientId=self.physics_client_id)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pd.getDataPath())
        p.changeDynamics(bodyUniqueId=self.robot, linkIndex=-1, mass=1.5)
        self.planeID = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("../woofh/urdf/woofh_d.urdf", [0, 0, 0.2], useMaximalCoordinates=False,
                                flags=p.URDF_USE_IMPLICIT_CYLINDER,baseOrientation=p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
        #  change the outlook
        self.woofh = Robot(self.robot, physics_client_id=self.physics_client_id)
        p.setPhysicsEngineParameter(
            numSolverIterations=300)
        self.reward_detail = np.array([0.] * 6, dtype=np.float32)
        self.step_num = 0
        self.leg.time_reset()
        self.leg.positions_control2(self.robot, [0, -np.pi/4, np.pi/2],[0,  -np.pi/4  , np.pi/2    ],
                                                [0, -np.pi/4, np.pi/2],[0,  -np.pi/4  , np.pi/2     ]  )

        self.pre_coorX= -0.04867
        self.pre_height = 0.1365



        while self.initial_count <100:
            self.initial_count+=1
            self.leg.positions_control2(self.robot, [0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2],
                                        [0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2])

            self.woofh.motor_angle = np.hstack(([0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2],
                                        [0, -np.pi / 4, np.pi / 2], [0, -np.pi / 4, np.pi / 2]))
            p.stepSimulation()

        self.initial_count = 0
        return self.get_observation()


    def render(self, mode='human'):
        pass

    def seed(self, seed=None):
        pass

    def _reward(self,reward_items):
        x_coor = reward_items[0]
        y_coor = reward_items[1]

        linearX = reward_items[2]
        linearY,linearZ  = reward_items[3:5]
        wx ,wy ,wz = reward_items[5:8]
        r , p , y  = reward_items[8:11]
        height = reward_items [11]

        contacts = reward_items[12:]
        contact_reward = -1.
        if contacts[0]==1 and contacts[2]==1 and contacts[1]==0 and contacts[3]==0:
            contact_reward = 1.
        if contacts[0]==0 and contacts[2]==0 and contacts[1]==1 and contacts[3]==1:
            contact_reward = 1.


        reward = self.forward_weightX *   (x_coor-self.pre_coorX) + \
                 -self.forward_weightY * (np.abs(y_coor)) + \
                 self.forwardV_weight *    linearX/4 + \
                 self.direction_weight * (linearY**2+linearZ**2)+ \
                 self.shake_weight*      ( np.exp( -1/(wx**2+wy**2+wz**2) ))+ \
                 self.shake_weight*      ( r**2/2+p**2/2+y**2)+ \
                 self.height_weight *    ( np.abs(height-self.pre_height)) +\
                 self.contact_weight* contact_reward

        # check every part rewards
        forward_reward = self.forward_weightX *   (x_coor-self.pre_coorX)
        forwardy_reward = -self.forward_weightY * np.abs(y_coor)
        forward_Vreward = self.forwardV_weight *   linearX/4
        direction_reward =   self.shake_weight*      ( np.exp( -1/(wx**2+wy**2+wz**2) ))+ self.shake_weight*  ( r**2/2+p**2/2+y**2)
        height_reward = self.height_weight *    ( np.abs(height-self.pre_height))
        contact_reard = self.contact_weight* contact_reward
        reward_details = np.array([forward_reward,forwardy_reward,forward_Vreward,direction_reward,height_reward,contact_reard])
        self.reward_detail+=reward_details


        if self.step_num % 10 ==0:
            self.pre_coorX = x_coor
        self.pre_height = height


        return reward

    def apply_action(self, action):
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in")


        # action_on_motor =self.merge_action(action)
        # self.leg.positions_control(self.robot, action_on_motor[0:3], action_on_motor[3:6],
        #                           action_on_motor[6:9], action_on_motor[9:12])

        if self.step_num >= 0 and self.step_num <= 20:
            random_force  = np.random.uniform(-12,12,3)
            p.applyExternalForce(objectUniqueId=self.robot, linkIndex=-1,
                             forceObj=[random_force[0], random_force[1],random_force[2]],
                             posObj=[-0.04072342, 0.00893663, 0.13637926],
                             flags=p.WORLD_FRAME
                             )

        x1,_,z1 = self.tg.curve_generator(self.leg.t1)
        x2, _, z2 = self.tg.curve_generator(self.leg.t2)
        theta1,theta2  =     self.leg.IK_2D(x1 ,-z1)
        theta3, theta4 =     self.leg.IK_2D(x2 ,-z2)

        self.angleFromReferen = np.array([0, theta1 ,theta2,0,theta3, theta4, 0, theta3 ,theta4, 0 , theta1 ,theta2])
        action_on_motor =  self.merge_action(action)
        self.leg.positions_control2(self.robot, action_on_motor[0:3], action_on_motor[3:6],
                                  action_on_motor[6:9], action_on_motor[9:12])

        self.woofh.motor_angle = action_on_motor
        # ---------------test for free control------------------------#
        # self.leg.positions_control2( self.robot, [0, theta2 ,theta3], [0,theta4, theta5],
        #                              [0,theta4, theta5], [0, theta2 ,theta3])
        self.leg.t1+= self.dt
        self.leg.t2+= self.dt




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

        LF = [0, 0, 0]
        RF = [0, 0, 0]
        LB = [0, 0, 0]
        RB = [0, 0, 0]

        # shoulder optimize signal from -5° to 5 °
        LF[0] = action[0] * self.opti_shoulder
        RF[0] = action[3] * self.opti_shoulder
        LB[0] = action[6] * self.opti_shoulder
        RB[0] = action[9] * self.opti_shoulder

        # hip,knee optimize signal from -15° to 15 °
        LF[1:] = action[1:3] * self.opti_kneeAhid
        RF[1:] = action[4:6] * self.opti_kneeAhid
        LB[1:] = action[7:9] * self.opti_kneeAhid
        RB[1:] = action[10:] * self.opti_kneeAhid

        action_on_motor = np.array([0] * 12)
        # print("----------------------------")
        # print(np.hstack((LF, RF , LB, RB)))
        # print(self.angleFromReferen)
        # print("****************************")
        return  np.hstack((LF, RF , LB, RB)) * self.optimSignal + self.angleFromReferen * self.referSignal


    def train_model(self,train_episode,save_episode,model):
        t1 = time.time()
        save_count = 1
        all_episode_reward = []
        for i in range(train_episode+1):
            episode_reward = 0
            t2 = time.time()
            obs = self.reset()
            t3 = time.time()
            while True:
                t4 = time.time()
                action = model.predict(obs)
                obs ,reward ,done, _ = self.step(action[0])
                episode_reward+=reward
                if done:
                    break
            t5 = time.time()
            print('episode_reward==={}'.format(episode_reward))
            all_episode_reward.append(episode_reward)
            print("train=={}".format(i))
            t6 = time.time()

            if i>1 and i %save_episode==0:
                model.save('gym_env/train_result'+str(save_count))
                save_count+=1

            print(t2-t1,t3-t2,t4-t3,t5-t4,t6-t5)

        file = open('gym_env/reward.txt','w')
        file.write(str(all_episode_reward))


    def test_model(self,test_model,test_speed,):
        done = False
        all_episode_reward = []
        for i in range(5):
            episode_reward = 0
            obs = self.reset()
            while True:
                time.sleep(test_speed)
                action = test_model.predict(obs)
                obs ,reward ,done, _ = self.step(action[0])
                episode_reward += reward
                if done:
                    break
            print("episode_reard==={}".format(episode_reward))
            all_episode_reward.append(episode_reward)
        return all_episode_reward


    def test_no_RL(self,test_round,test_speed,model):
        done = False
        self.optimSignal = 0
        all_episode_reward = []
        for i in range(test_round):
            obs = self.reset()
            episode_reward = 0
            while True:
                time.sleep(test_speed)
                action = model.predict(obs)
                obs, reward, done, _ = self.step(action[0])
                episode_reward += reward
                if done:
                    break
            all_episode_reward.append(episode_reward)
            print( self.woofh.get_Global_Coor() )

        return all_episode_reward



if __name__ == '__main__':
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3 import PPO
    from stable_baselines3.common.evaluation import  evaluate_policy


    device = [torch.cuda.device(i) for i in range((torch.cuda.device_count()))]
    print(device)


    env = Dog(render=True)
    model = PPO(policy = "MlpPolicy",env = env,device='cuda',batch_size=2048,verbose=1)
    #
    # model.learn(500000)
    # model.save('gym_env/train_result_2')

    reward = env.test_no_RL(10,0.001,model)


    print(reward)
    # loaded_model = PPO.load('gym_env/train_result_2')
    # mean_reward, std_reward = evaluate_policy(loaded_model, env, n_eval_episodes=10)
    # print(f"mean_reward:{mean_reward:.2f} +/- {std_reward:.2f}")


    # model.load('gym_env/train_result')
    # reward = env.test_model(test_model=model,test_speed=0.01)
    # print(reward)

    # t1 = time.time()
    # model.learn(1000000)
    # model.save('gym_env/train_result')


    # # env.train_model(10,10)
    #
    # t2 = time.time()
    #
    # print(t2-t1)




    # all_rewards = env.test_no_RL(5,0.01)
    # print(all_rewards)
    # env.test_no_RL(10,0.01)
    # t1 = time.time()
    # for i in range(10000):
    #     episode_reward = 0
    #     obs = env.reset()
    #     done = False
    #     while True:
    #         action = model.predict(obs)
    #         obs ,reward ,done, _ = env.step(action[0])
    #         episode_reward+=reward
    #         if done:
    #             break
    #     print('episode_reward==={}'.format(episode_reward))
    #     all_episode_reward.append(episode_reward)
    #     print("train=={}".format(i))
    #
    #     if i %50==0:
    #         print('episode_reward==={}'.format(episode_reward))
    #
    # file = open('gym_env/reward.txt','w')
    # file.write(str(all_episode_reward))
    # model.save('gym_env/train_result')






    # t2 = time.time()
    # print(t2-t1)
    # check_env(env)
