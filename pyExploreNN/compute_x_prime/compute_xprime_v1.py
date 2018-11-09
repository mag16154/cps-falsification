import sys
sys.path.append('../Configuration_setup/')
import numpy as np
from NNConfiguration import NNConfiguration
from itertools import combinations

## Input (x,t)
## Output x'
## x and x' are taken at consecutive time steps

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

        input = []
        output = []
        for traj_idx in range(len(self.trajectories) - 1):
            for step in range(self.steps):
                x_t_pair = list(self.trajectories[traj_idx][0])
                x_t_pair.append(step*self.timeStep)
                input.append(x_t_pair)
                output.append(self.trajectories[traj_idx][step])
        print(len(input))
        self.input = np.asarray(input)
        self.output = np.asarray(output)
        # scalar = StandardScaler()
        # scalar.fit(input)
        # input = scalar.transform(input)
        # print(len(output))
        # print(output)
        # scalar.fit(output)
        # output = scalar.transform(output)
        # print(output)
        # print(outputs)