%% 3 dimensional nonlinear system with dynamics
%% dxdt = y; dydt = z*z*sin(x)*cos(x) - sin(x) - epsilon*y; dzdt = alpha*(cos(x) - beta)


clear all;
clc;

addpath('./bulid_and_test_NN/');

no_of_dims = 3
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
[inputSeries, targetSeries] = createNNInput(traj_x, time_steps, no_of_dims, traj_combs);
epochs =100;
layers = 2;
neurons = zeros(1, layers);
neurons(1, 1) = 25;
neurons(1, 2) = 20;
preprocess = false
[net, output_mat, target_mat] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons, layers, preprocess);
%v_rae = validateNN(net, traj_x, time_steps, no_of_dims, traj_combs, layers, preprocess);

plotFigures(output_mat, target_mat, no_of_dims, 'Steam');

% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set

epsilon = 3;
alpha = 1;
beta = 1;
%%% variables

x=v(1);
y=v(2);
z=v(3);
%%% equations
%dxdt = y; dydt = z*z*sin(x)*cos(x) - sin(x) - epsilon*y; dzdt = alpha*(cos(x) - beta)
dv = [
    y;  % dx/dt
    z*z*sin(x)*cos(x) - sin(x) - epsilon*y;  	% dy/dt
    alpha*(cos(x) - beta); %dz/dt
] ;
end
