import sys
sys.path.append('../NNConfiguration_setup/')
import numpy as np
from NNConfiguration_v2 import NNConfiguration
from itertools import combinations
import random

# Input (x, v, t)
# Output v'
# x is at 0 and v are taken at consecutive times starting from 1
# The test set is created explicitly


class nnOnStateSpace(NNConfiguration):

    def __init__(self, timeStep=0.01, steps=100, samples=50, dynamics='None', dimensions=2, lowerBound=[], upperBound=[]):

        NNConfiguration.__init__(self, timeStep, steps, samples, dynamics, dimensions, lowerBound, upperBound)

    def createNN(self, jump_size=1):

        assert self.lowerBoundArray is not [] and self.upperBoundArray is not []

        traj_combs = []

        end_idx = round(self.samples*3/4)
        start_idx = 0
        traj_indices = list(range(start_idx, end_idx))
        traj_combs += list(combinations(traj_indices, 2))
        print(traj_indices)

        input = []
        output = []
        steps = len(self.trajectories[traj_combs[0][0]])-1
        for traj_pair in traj_combs:
            t_pair = list(traj_pair)
            traj_1 = self.trajectories[t_pair[0]]
            traj_2 = self.trajectories[t_pair[1]]
            for step in range(1, steps, jump_size):
                x_idx = 0
                x_v_t_pair = list(traj_1[x_idx])
                v_val = traj_2[x_idx] - traj_1[x_idx]
                t_val = step
                # t_idx = random.randint(0, steps-step)
                # t_val = step+t_idx
                vprime_val = traj_2[t_val] - traj_1[t_val]
                x_v_t_pair = x_v_t_pair + list(v_val)
                x_v_t_pair = x_v_t_pair + [t_val*self.timeStep]
                input.append(x_v_t_pair)
                output.append(vprime_val)
        print(len(input))
        self.input = np.asarray(input)
        self.output = np.asarray(output)

        traj_combs = []
        start_idx = end_idx
        end_idx = self.samples
        traj_indices = list(range(start_idx, end_idx))
        traj_combs += list(combinations(traj_indices, 2))
        print(traj_indices)
        x_test = []
        y_test = []
        for traj_pair in traj_combs:
            x_test_local = []
            y_test_local = []
            t_pair = list(traj_pair)
            traj_1 = self.trajectories[t_pair[0]]
            traj_2 = self.trajectories[t_pair[1]]
            for step in range(1, steps, jump_size):
                x_idx = 0
                x_v_t_pair = list(traj_1[x_idx])
                v_val = traj_2[x_idx] - traj_1[x_idx]
                t_val = step
                # t_idx = random.randint(0, steps-step)
                # t_val = step+t_idx
                vprime_val = traj_2[t_val] - traj_1[t_val]
                x_v_t_pair = x_v_t_pair + list(v_val)
                x_v_t_pair = x_v_t_pair + [t_val*self.timeStep]
                x_test_local.append(x_v_t_pair)
                y_test_local.append(vprime_val)
            x_test.append(x_test_local)
            y_test.append(y_test_local)

        self.x_test = x_test
        self.y_test = y_test

