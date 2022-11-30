# Fachhochschule Aachen
# Name:Bruce_Xing
# Time: 2022/11/29 11:35

import torch
from  torch import nn
import torch.nn.functional as F
import numpy as np

class FeedForwardNN(nn.Module):
    def __init__(self,input_dim, output_dim):
        super(FeedForwardNN, self).__init__()
        self.layer1 = nn.Linear(input_dim,64)
        self.layer2 = nn.Linear(64,128)
        self.layer3 = nn.Linear(128,64)
        self.layer4 = nn.Linear(64,output_dim)

    def forward(self,obs):
        if isinstance(obs,np.ndarray):
            obs = torch.tensor(obs,dtype=torch.float)

        activation1 = F.relu(self.layer1(obs))
        activation2 = F.relu(self.layer2(activation1))
        activation3 = F.relu(self.layer3(activation2))
        output = self.layer4(activation3)
        return output

