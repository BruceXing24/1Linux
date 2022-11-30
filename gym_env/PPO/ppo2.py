# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/29 11:35
from network import FeedForwardNN
import torch
from torch.distributions import MultivariateNormal
from  torch import nn
import gym

class ppo():
    def __init__(self,env):
        self.env = env
        self._init_hyperparameters()
        self.obs_dim  = env.observation_space.shape[0]
        self.act_dim  = env.action_space.shape[0]

        self.actor = FeedForwardNN(self.obs_dim,self.act_dim)
        self.critic = FeedForwardNN(self.obs_dim,1)

        self.cov_var = torch.full(size=(self.act_dim,),fill_value=0.5)
        self.cov_mat = torch.diag(self.cov_var)
        self.gamma = 0.99
        self.actor_optim = torch.optim.Adam(self.actor.parameters(),lr = self.lr)
        self.critic_optim = torch.optim.Adam(self.critic.parameters(),lr = self.lr)

    def learn(self,total_timesteps):
        t_so_far = 0
        while t_so_far <total_timesteps:

            t_so_far +=1

            batch_obs, batch_acts, batch_log_probs, batch_rtgs,batch_lens = self.rollout()

            V,_ = self.evaluate(batch_obs,batch_acts)

            A_k = batch_rtgs-V.detach()

            A_k = (A_k-A_k.mean())/(A_k.std()+1e-10)

            for _ in range(self.n_updates_per_iteration):
                #calculate pi_theta(a_t|s_t)
                V, curr_log_probs = self.evaluate(batch_obs,batch_acts)

                ratios = torch.exp(curr_log_probs-batch_log_probs)

                #calculate surrogate losses
                surr1 = ratios*A_k
                surr2 = torch.clamp(ratios, 1-self.clip,1+self.clip) *A_k

                actor_loss = (-torch.min(surr1,surr2)).mean()
                critic_loss = nn.MSELoss()(V, batch_rtgs)


                self.actor_optim.zero_grad()
                actor_loss.backward( retain_graph = True )
                self.actor_optim.step()

                self.critic_optim.zero_grad()
                critic_loss.backward()
                self.critic_optim.step()

            print("第{}轮训练".format(t_so_far))

    def get_action(self,obs):
        mean = self.actor(obs)
        dist = MultivariateNormal(mean,self.cov_mat)
        action = dist.sample()
        log_prob = dist.log_prob(action)
        return action.detach().numpy(), log_prob.detach()

    def compute_rtgs(self,batch_rews):
        batch_rtgs = []

        for ep_rews in reversed(batch_rews):
            discount_reward = 0
            for rew in reversed(ep_rews):
                discount_reward = rew +discount_reward* self.gamma
                batch_rtgs.insert(0,discount_reward)
        batch_rtgs = torch.tensor(batch_rtgs,dtype=torch.float)
        return batch_rtgs

    def evaluate(self,batch_obs,batch_acts):
        V = self.critic(batch_obs).squeeze()
        mean = self.actor(batch_obs)
        dist = MultivariateNormal(mean,self.cov_mat)
        log_probs = dist.log_prob(batch_acts)

        return V,log_probs


    def _init_hyperparameters(self):
        self.timesteps_per_batch = 4800
        self.max_timesteps_per_episode = 1600
        self.n_updates_per_iteration = 5
        self.clip  = 0.2
        self.lr  =  0.01



    def rollout(self):
        batch_obs = []
        batch_acts = []
        batch_log_probs = []
        batch_rews = []
        batch_reward_to_go = []
        batch_lens = []
        t = 0
        while t < self.timesteps_per_batch:
            ep_rews = []
            obs = self.env.reset()
            done = False
            for ep_t in range(self.max_timesteps_per_episode):
                t += 1
                batch_obs.append(obs)
                action , log_prob = self.get_action(obs)
                obs,rew,done,_    = self.env.step(action)
                ep_rews.append(rew)
                batch_acts.append(action)
                batch_log_probs.append(log_prob)
                if done:
                    break

            batch_lens.append(ep_t+1)
            batch_rews.append(ep_rews)

        batch_obs = torch.tensor(batch_obs,dtype=torch.float)
        batch_acts = torch.tensor(batch_acts, dtype=torch.float)
        batch_log_probs = torch.tensor(batch_log_probs, dtype=torch.float)

        batch_rtgs = self.compute_rtgs(batch_rews)
        return batch_obs,batch_acts,batch_log_probs,batch_rtgs,batch_lens

if __name__ == '__main__':
    env = gym.make('Pendulum-v1')
    model = ppo(env)
    model.learn(1000)