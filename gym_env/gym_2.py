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
        self.forward_weightX = 0.05
        self.forward_weightY = 0.02
        self.forwardV_weight = 0.01
        self.direction_weight = -0.001
        self.shake_weight = -0.005
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
        self.referSignal  = 0.5
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
        #
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







if __name__ == '__main__':
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3 import PPO
    env = Dog(render=True)
    model = PPO(policy = "MlpPolicy",env = env)
    # p.setRealTimeSimulation(1)
    all_episode_reward = []
    episode_reward = 0
    t1 = time.time()
    for i in range(1000):
        episode_reward = 0
        obs = env.reset()
        done = False
        while True:
            time.sleep(0.01)
            action = model.predict(obs)
            print(action)
            # print(action[1])
            # action = np.array(action,dtype=object).reshape(-1)
            print("obs==={}".format(obs))
            obs ,reward ,done, _ = env.step(action[0])
            episode_reward+=reward
            if done:
                break

        all_episode_reward.append(episode_reward)
        print("train=={}".format(i))

        if i %50==0:
            print('episode_reward==={}'.format(episode_reward))

    # file = open('gym_env/reward.txt','w')
    # file.write(str(all_episode_reward))
    # model.save('gym_env/train_result')


    # model.load('gym_env/train_result')
    # obs = env.reset()
    # while True:
    #     time.sleep(0.01)
    #     action = model.predict(obs)
    #     action = np.array(action,dtype=object)
    #     state ,reward ,done, _ = env.step(action)
    #     if done:
    #         break




    # t2 = time.time()
    # print(t2-t1)
    # check_env(env)
