import matplotlib.pyplot as plt
from configuration import configuration
from frechet import normTrajectory, norm
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from analyzer import visualizePerturbations
from sklearn.metrics import mean_squared_error


class NNConfiguration(configuration):

    def __init__(self, timeStep=0.01, steps=100, samples=10, dynamics='None', dimensions=2, lowerBound=[], upperBound=[]):

        configuration.__init__(self, timeStep, steps, samples, dynamics, dimensions, lowerBound, upperBound)
        self.relativeError = []
        self.sortedError = []
        self.input = None
        self.output = None
        self.no_of_layers = 2
        self.neurons = [30, 30]
        self.test_size = 0.33

    def generateTrajectories(self):
        self.storeTrajectories()

    def setLayers(self, number_of_layers):
        self.no_of_layers = number_of_layers

    def setNeurons(self, neurons):
        self.neurons = neurons

    def trainTestNN(self, act_fn='relu', solver_fn='adam'):

        X_train, X_test, Y_train, Y_test = train_test_split(self.input, self.output, test_size=self.test_size, random_state=1)
        print(X_train[0])
        print(X_test[0])
        clf = None
        if self.no_of_layers == 2:
            clf = MLPRegressor(activation = act_fn, solver= solver_fn, alpha=1e-5, hidden_layer_sizes=(self.neurons[0], self.neurons[1]), random_state=1)
        elif self.no_of_layers == 3:
            clf = MLPRegressor(activation = act_fn, solver= solver_fn, alpha=1e-5, hidden_layer_sizes=(self.neurons[0], self.neurons[1], self.neurons[2]),
                               random_state=1)

        clf.fit(X_train, Y_train)
        test_predictions = clf.predict(X_test)
        print("Y_test length {} test_predictions length {}".format(len(Y_test), len(test_predictions)))

        mse = mean_squared_error(Y_test, test_predictions)
        print("Mean Squared error {}".format(mse))
        # mseError = 0.0
        # for idx in range(0, len(test_predictions)):
        #    mseError += mean_squared_error(test_predictions[idx], Y_test[idx])
        # mseError = mseError/len(test_predictions)
        # print("Manual MSE {}".format(mseError))
        relativeError = []
        for idx in range(0, len(test_predictions)):
            distVal = norm(test_predictions[idx] - Y_test[idx], -1)
            relativeError += [distVal/norm(Y_test[idx], -1)]
        plt.plot(relativeError)
        print("Mean relative error {}".format(sum(relativeError)/len(test_predictions)))
        self.relativeError = [relativeError]
        # sortedError = []
        # for idx in range(0, len(test_predictions)):
        #     sortedError += [norm(test_predictions[idx], -1)/norm(Y_test[idx], -1)]
        # plt.plot(sortedError)
        # self.sortedError = [sortedError]
        # print("Mean sorted error {}".format(sum(sortedError) / len(test_predictions)))
        plt.show()

    def showRelativePerturbation(self):

        visualizePerturbations(self.relativeError)

    def showApproxPerturbation(self):

        visualizePerturbations(self.sortedError)
