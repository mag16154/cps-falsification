import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import random as rand

from sampler import generateRandomStates, generateSuperpositionSampler
from ODESolver import generateTrajectories, plotTrajectories

class configuration(object):
	def __init__(self, timeStep=0.01, steps=100, samples=50, dynamics='None', dimensions=2, lowerBound=[],
				 upperBound=[], embedding_dimension=2):
		self.timeStep = timeStep
		self.steps = steps
		self.samples = samples
		self.dynamics = dynamics
		self.dimensions = dimensions
		self.lowerBoundArray = lowerBound
		self.upperBoundArray = upperBound
		self.trajectories = []
		self.states = []

		# Only these are in the configuration.
		# Getting trajectories is the duty of configuration.


	def setDynamics(self, dynamics):
		self.dynamics = dynamics

	def setLowerBound(self, lowerBound):
		self.lowerBoundArray = lowerBound

	def setUpperBound(self, upperBound):
		self.upperBoundArray = upperBound

	def setSamples(self, samples):
		self.samples = samples

	def setSteps(self, steps):
		self.steps = steps

	def setTimeStep(self, timeStep):
		self.timeStep = timeStep

	def setEmbeddingDimension(self, dim):
		self.embedding_dimension = dim

	def storeStatesRandomSample(self):
		self.states = generateRandomStates(self.samples, self.lowerBoundArray, self.upperBoundArray)

	def storeTrajectories(self):

		if self.states == [] :
			self.storeStatesRandomSample()

		assert not (self.states == [])
		#assert self.lowerBoundArray is not [] and self.upperBoundArray is not []
		self.time = np.linspace(0, self.timeStep * self.steps, self.steps + 1)
		self.trajectories = generateTrajectories(self.dynamics, self.states, self.time)

	def showTrajs(self, xindex=0, yindex=1):
		plotTrajectories(self.trajectories, xindex=xindex, yindex=yindex)



#config1 = configuration()
# time step default = 0.01, number of steps default = 100
# dimensions default = 2, number of sample default = 50

#config1.setSteps(100)
#config1.setSamples(10)

#config1.setDynamics('Vanderpol')
#config1.setLowerBound([1.0, 1.0])
#config1.setUpperBound([50.0, 50.0])

# config1.setDynamics('Brussellator')
# config1.setLowerBound([1.0, 1.0])
# config1.setUpperBound([2.0, 2.0])

# config1.setDynamics('Lorentz')
# config1.setLowerBound([1.0, 1.0, 1.0])
# config1.setUpperBound([10.0, 10.0, 10.0])

#for i in range(0, 1):

#	config1.storeTrajectories()
#	config1.rawAnalyze()
#	print (config1.eigenElbow)
#	print (config1.noProminetEigVals)

#	for j in range(0, config1.noProminetEigVals):
#		print (config1.sortedVectors[j])
#	config1.showLogEigenValues()
#	config1.showTrajs()
