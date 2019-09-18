import matplotlib.pyplot as plt
from configuration import configuration
from frechet import normTrajectory, norm
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from analyzer import visualizePerturbations
from sklearn.metrics import mean_squared_error


class NNConfiguration(configuration):

    def __init__(self, timeStep=0.01, steps=100, samples=50, dynamics='None', dimensions=2, lowerBound=[], upperBound=[]):

        configuration.__init__(self, timeStep, steps, samples, dynamics, dimensions, lowerBound, upperBound)
        self.relativeError = []
        self.mseError = []
        self.input = None
        self.output = None
        self.x_test = None
        self.y_test = None
        self.no_of_layers = 2
        self.neurons = [30, 30]
        self.test_size = 0.0

    def generateTrajectories(self):
        self.storeTrajectories()

    def setLayers(self, number_of_layers):
        self.no_of_layers = number_of_layers

    def setNeurons(self, neurons):
        self.neurons = neurons

    def trainTestNN(self, act_fn='relu', solver_fn='adam'):

        scale = 2

        print(len(self.trajectories))
        # plt.figure()
        for idx in range(len(self.trajectories)):
            traj = self.trajectories[idx]
            norm_vals = []
            for idy in range(round(len(traj)/scale)):
                norm_val = norm(traj[idy], -1)
                norm_vals += [norm_val]
            # plt.plot(norm_vals)

        # plt.show()
        X_train, X_test, Y_train, Y_test = train_test_split(self.input, self.output, test_size=self.test_size,

                                                            random_state=1)
        print(len(X_train))
        print(len(X_test))
        self.no_of_layers = len(self.neurons)
        h_layer_sizes = (self.neurons[0],)
        for n_layer in range(1, self.no_of_layers):
            h_layer_sizes += (self.neurons[n_layer],)

        clf = MLPRegressor(activation = act_fn, solver= solver_fn, alpha=1e-5, hidden_layer_sizes=h_layer_sizes,
                           random_state=1)

        clf.fit(X_train, Y_train)

        plt.figure()
        # plt.subplot(3, 1, 2)
        mse_global_sum = 0.0
        for idx in range(round(len(self.x_test)/scale)):
            X_test = self.x_test[idx]
            Y_test = self.y_test[idx]
            test_predictions = clf.predict(X_test)
            # print("Y_test length {} test_predictions length {}".format(len(Y_test), len(test_predictions)))

            mse = mean_squared_error(Y_test, test_predictions)
            # print("Mean Squared error {}".format(mse))
            mse_global_sum = mse_global_sum + mse

            mseError = []
            for idx in range(0, round(len(test_predictions)/scale)):
                mse = mean_squared_error(test_predictions[idx], Y_test[idx])
                mseError += [mse]
            plt.plot(mseError)
        plt.show()

        print(mse_global_sum)
        print(round(len(self.x_test)/scale))
        mse_global_av = mse_global_sum/round(len(self.x_test)/scale)
        print("Mean Squared error global {}".format(mse_global_av))
        plt.figure()
        # plt.subplot(3, 1, 3)
        relErr_global_sum = 0.0
        for idx in range(round(len(self.x_test)/scale)):
            X_test = self.x_test[idx]
            Y_test = self.y_test[idx]
            test_predictions = clf.predict(X_test)

            relError = []

            relErrorSum = 0.0
            for idx in range(0, round(len(test_predictions)/scale)):
                distVal = norm(test_predictions[idx] - Y_test[idx], -1)
                relErr = (distVal/(norm(test_predictions[idx], -1)))
                relError += [relErr]
                relErrorSum += relErr
            rerr_avg = relErrorSum/round(len(test_predictions/scale))
            # print("Relative error {}".format(rerr_avg))
            relErr_global_sum = relErr_global_sum+rerr_avg
            plt.plot(relError)
        plt.show()
        relErr_global_av = relErr_global_sum/round(len(self.x_test)/scale)
        print("Relative error global {}".format(relErr_global_av))

    def showRelativePerturbation(self):

        visualizePerturbations(self.relativeError)

    def showApproxPerturbation(self):

        visualizePerturbations(self.sortedError)
