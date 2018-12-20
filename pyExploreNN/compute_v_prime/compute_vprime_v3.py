import sys
sys.path.append('../NNConfiguration_setup/')
import numpy as np
from NNConfiguration import NNConfiguration
from itertools import combinations
import random

## Input (x,v,x',t)
## Output v'
## x are taken at consecutive times but x' is picked at a random time

class nnOnStateSpace(NNConfiguration):

    def __init__(self, timeStep=0.01, steps=100, samples=50, dynamics='None', dimensions=2, lowerBound=[], upperBound=[]):

        NNConfiguration.__init__(self, timeStep, steps, samples, dynamics, dimensions, lowerBound, upperBound)

    def createNN(self, jump_size=1):

        assert self.lowerBoundArray is not [] and self.upperBoundArray is not []

        traj_combs = []

        for jump in range(1, jump_size+1):
            start_idx = (jump-1)*10
            end_idx = jump*10
            traj_indices = list(range(start_idx, end_idx))
            traj_combs += list(combinations(traj_indices, 2))

        print(jump_size)
        input = []
        output = []
        steps = len(self.trajectories[traj_combs[0][0]])-1
        for traj_pair in traj_combs:
            t_pair = list(traj_pair)
            traj_1 = self.trajectories[t_pair[0]]
            traj_2 = self.trajectories[t_pair[1]]
            for step in range(0, steps, jump_size):
                x_v_xp_t_pair = list(traj_1[step])
                v_val = traj_2[step] - traj_1[step]
                t_idx = random.randint(0, steps-step)
                t_val = step+t_idx
                vprime_val = traj_2[t_val] - traj_1[t_val]
                x_v_xp_t_pair = x_v_xp_t_pair + list(v_val)
                x_v_xp_t_pair = x_v_xp_t_pair + list(traj_1[t_val])
                x_v_xp_t_pair.append(t_val)
                input.append(x_v_xp_t_pair)
                output.append(vprime_val)
        print(len(input))
        self.input = np.asarray(input)
        self.output = np.asarray(output)