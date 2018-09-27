# cps-falsification
Code commits related to our work on CPS Falsification.

a) matlabNaive folder contains the prelimary work where we manually tried to find the direction for basis vectors based on the distance metric.

(File Naming convention) 
# 06-22-2017
1) A numeral added as a suffix to the keyword 'linear' is to distinguish among benchmarks with linear dynamics.
2) 2d is for a 2 dimensional system.
3) iter1 represents the code for arrving at an initial state in a large initial set, around which we need to compute the robustness in second iteration.
4) iter2 is for the iteration where we change values along one dimension and keep the values along other dimensions intact.

b) matlabNN folder corresponds to using neural network to explore the state space.
