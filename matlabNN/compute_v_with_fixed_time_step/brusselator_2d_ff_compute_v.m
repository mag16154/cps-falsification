%% 2 dimensional nonlinear system with dynamics
%% x = A + x * x * y - B * x - x
%% y = B * x - x * x * y

clear all;
clc;

addpath('./bulid_and_test_NN/');

no_of_dims = 2
no_of_trajs = 10
traj_combs = combnk(1:no_of_trajs,2)
[no_of_samples, dim2] = size(traj_combs);
init_state_array = rand(no_of_samples,no_of_dims)*10

%% Time

tspan = 0:0.01:10;

%% Integration/Simulation

for idx = 1:no_of_samples
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	traj_t(:,idx) = t;
	traj_x(:,:,idx) = x;
end

[time_steps elems] = size(traj_t);

layers = 2;
neurons = zeros(1, layers);
neurons(1, 1) = 10;
neurons(1, 2) = 10;
preprocess = false;
epochs = 50;
for idx = 0:1
	jump = 2.^idx;
	[inputSeries, targetSeries] = createNNInput(traj_x, time_steps, no_of_dims, traj_combs, jump);
	[net, output_mat, target_mat] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons, layers, preprocess);
	[v_mse] = validateNN(net, traj_x, time_steps, no_of_dims, traj_combs, layers, preprocess);
end
plotFigures(output_mat, target_mat, no_of_dims, 'Brusselator');

	
% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set 1

A=1;
B=1.5;

%%% variables

x=v(1);
y=v(2);
%%% equations

dv = [
    A + x*x*y - B*x - x;  % dx/dt
    B*x-x*x*y;  	% dy/dt
] ;
end
