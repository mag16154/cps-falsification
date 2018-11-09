import sys
sys.path.append('../Configuration_setup/')
import numpy as np
from NNConfiguration import NNConfiguration
from itertools import combinations
import random


class nnOnStateSpace(NNConfiguration):

    def __init__(self, timeStep=0.01, steps=100, samples=50, dynamics='None', dimensions=2, lowerBound=[], upperBound=[]):

        NNConfiguration.__init__(self, timeStep, steps, samples, dynamics, dimensions, lowerBound, upperBound)

    def rescaleColumn(self, col, col_min, col_max):
        col = list(col)
        for i in range(len(col)):
            col[i] = (col[i]*(col_max-col_min)) + col_min
        return col

    def scaleColumn(self, col):
        col = list(col)
        col_min = min(col)
        col_max = max(col)
        for i in range(len(col)):
            col[i] = (col[i]-col_min)/(col_max-col_min)
        return col

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
                x_t_pair.append(step * self.timeStep)
                input.append(x_t_pair)
                output.append(self.trajectories[traj_idx][step])
        print(len(input))

        col1, col2, col3 = zip(*input)
        s_col1 = self.scaleColumn(col1)
        s_col2 = self.scaleColumn(col2)
        s_col3 = self.scaleColumn(col3)

        l = [s_col1, s_col2, s_col3]    # l has all the scaled input values
        l = list(map(list,list(zip(*l))))
        input = l

        self.input = np.asarray(input)

        col4, col5 = zip(*output)
        s_col4 = self.scaleColumn(col4)
        s_col5 = self.scaleColumn(col5)

        l = [s_col4, s_col5]    # l has all the scaled input values
        l = list(map(list,list(zip(*l)))) #
        output = l
        self.output = np.asarray(output)

    def trainTestNN(self, t_size, no_of_layers, neurons, act_fn, solver_fn):

        X_train, X_test, Y_train, Y_test = train_test_split(self.input, self.output, test_size=t_size, random_state=1)
        clf = None
        if no_of_layers == 2:
            clf = MLPRegressor(activation=act_fn, solver=solver_fn, alpha=1e-5,
                                hidden_layer_sizes=(neurons[0], neurons[1]), random_state=1)
        elif no_of_layers == 3:
            clf = MLPRegressor(activation=act_fn, solver=solver_fn, alpha=1e-5,
                                hidden_layer_sizes=(neurons[0], neurons[1], neurons[2]), random_state=1)

        clf.fit(X_train, Y_train)
        train_predictions = clf.predict(X_train)
        test_predictions = clf.predict(X_test)
        print("Y_test length {} test_predictions length {}".format(len(Y_test), len(test_predictions)))

        # Rescale input and output
        s_col1, s_col2, s_col3 = zip(*X_train)  # Rescale the X_train data
        s_col1 = self.rescaleColumn(s_col1, min(col1), max(col1))
        s_col2 = self.rescaleColumn(s_col2, min(col2), max(col2))
        s_col3 = self.rescaleColumn(s_col3, min(col3), max(col3))

        l = [s_col1, s_col2, s_col3]
        l = list(map(list, list(zip(*l))))
        rescaled_X_train = np.array(l)

        s_col1, s_col2, s_col3 = zip(*X_test)
        s_col1 = self.rescaleColumn(s_col1, min(col1), max(col1))
        s_col2 = self.rescaleColumn(s_col2, min(col2), max(col2))
        s_col3 = self.rescaleColumn(s_col3, min(col3), max(col3))

        l = [s_col1, s_col2, s_col3]
        l = list(map(list, list(zip(*l))))
        rescaled_X_test = np.array(l)

        s_col4, s_col5 = zip(*Y_train)
        s_col4 = self.rescaleColumn(s_col4, min(col4), max(col4))
        s_col5 = self.rescaleColumn(s_col5, min(col5), max(col5))

        l = [s_col4, s_col5]
        l = list(map(list, list(zip(*l))))
        rescaled_Y_train = np.array(l)

        s_col4, s_col5 = zip(*Y_test)
        s_col4 = self.rescaleColumn(s_col4, min(col4), max(col4))
        s_col5 = self.rescaleColumn(s_col5, min(col5), max(col5))

        l = [s_col4, s_col5]
        l = list(map(list, list(zip(*l))))
        rescaled_Y_test = np.array(l)

        s_col4, s_col5 = zip(*train_predictions)
        s_col4 = self.rescaleColumn(s_col4, min(col4), max(col4))
        s_col5 = self.rescaleColumn(s_col5, min(col5), max(col5))

        l = [s_col4, s_col5]
        l = list(map(list, list(zip(*l))))
        rescaled_train_predictions = np.array(l)

        s_col4, s_col5 = zip(*test_predictions)
        s_col4 = self.rescaleColumn(s_col4, min(col4), max(col4))
        s_col5 = self.rescaleColumn(s_col5, min(col5), max(col5))

        l = [s_col4, s_col5]
        l = list(map(list, list(zip(*l))))
        rescaled_test_predictions = np.array(l)

        Training_rescaled_mse = mean_squared_error(rescaled_Y_train, rescaled_train_predictions)
        print('Training rescaled mse', Training_rescaled_mse)
        rescaled_mse = mean_squared_error(rescaled_Y_test, rescaled_test_predictions)
        print('rescaled mean sq error', rescaled_mse)

        errors_train = abs(rescaled_train_predictions - rescaled_Y_train)
        mre_train = np.mean(errors_train / abs(rescaled_Y_train))
        print('MRE_training:', mre_train)

        errors_test = abs(rescaled_test_predictions - rescaled_Y_test)
        mre_test = (np.mean(errors_test / abs(rescaled_Y_test)) * 100)
        print('MRE_testing:', round(mre_test, 2), '%.\n')


        relativeError = []
        for idx in range(0, len(test_predictions)):
            distVal = norm(test_predictions[idx] - Y_test[idx], -1)
            relativeError += [distVal / norm(Y_test[idx], -1)]
        plt.plot(relativeError)

        self.relativeError = [relativeError]
        sortedError = []
        for idx in range(0, len(test_predictions)):
            sortedError += [norm(test_predictions[idx], -1) / norm(Y_test[idx], -1)]
        plt.plot(sortedError)
        self.sortedError = [sortedError]
        plt.show()

