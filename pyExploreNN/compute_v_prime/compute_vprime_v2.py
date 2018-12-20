import sys
sys.path.append('../NNConfiguration_setup/')
import numpy as np
from NNConfiguration import NNConfiguration
from itertools import combinations

## Input (x,v,x')
## Output v'
## x and x' are taken at jump times
## We also add samples for every increase in jump_size in order to
## keep the number of input/output pairs same across each jump_size run

class nnOnStateSpace(NNConfiguration):

    def __init__(self, timeStep=0.01, steps=100, samples=50, dynamics='None', dimensions=2, lowerBound=[], upperBound=[]):

        NNConfiguration.__init__(self, timeStep, steps, samples, dynamics, dimensions, lowerBound, upperBound)

    def createNN(self, jump_size=1):

        assert self.lowerBoundArray is not [] and self.upperBoundArray is not []

        traj_combs = []

        for jump in range(1, jump_size+1):
            start_idx = (jump-1)*10 + 1
            end_idx = jump*10 + 1
            traj_indices = list(range(start_idx, end_idx))
            traj_combs += list(combinations(traj_indices, 2))

        print(jump_size)
        input = []
        output = []
        steps = len(self.trajectories[traj_combs[0][0]])
        for traj_pair in traj_combs:
            t_pair = list(traj_pair)
            traj_1 = self.trajectories[t_pair[0]]
            traj_2 = self.trajectories[t_pair[1]]
            for step in range(0, steps-jump_size, jump_size):
                x_v_xp_pair = list(traj_1[step])
                v_val = traj_2[step] - traj_1[step]
                vprime_val = traj_2[step+jump_size] - traj_1[step+jump_size]
                x_v_xp_pair = x_v_xp_pair + list(v_val)
                x_v_xp_pair = x_v_xp_pair + list(traj_1[step+jump_size])
                #print(x_v_xprime_pair)
                input.append(x_v_xp_pair)
                output.append(vprime_val)
        print(len(input))
        self.input = np.asarray(input)
        self.output = np.asarray(output)