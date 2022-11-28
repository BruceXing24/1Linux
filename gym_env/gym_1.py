# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/21 12:42


import gym
import pybullet as p
import pybullet_envs
from time import sleep

#---------普通的gym环境------------#
# env = gym.make("CartPole-v0")
# env.reset()
#
# for i in range(100):
#     env.render()
#     act = env.action_space.sample()
#     obs, reward, done, _ = env.step(act)
#
# env.close()

cid = p.connect(p.DIRECT)
env = gym.make("CartPoleContinuousBulletEnv-v0")
env.render()
env.reset()

for i in range(100):
    sleep(1/50)
    action = env.action_space.sample()
    obs, reward, done, _ = env.step(action)
p.disconnect(cid)


