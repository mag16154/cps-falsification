%% 3 dimensional nonlinear system with dynamics
%% dxdt = -y-z; dydt = x+a*y; dzdt = b + z*(x-c)

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
epochs =10;
layers = 2;
neurons = zeros(1, layers);
neurons(1, 1) = 10;
neurons(1, 2) = 10;
preprocess = false
[net, output_mat, target_mat] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons, layers, preprocess);
%v_rae = validateNN(net, traj_x, time_steps, no_of_dims, traj_combs, layers, preprocess);

plotFigures(output_mat, target_mat, no_of_dims, 'Roesseler');

	
% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set

a = 0.2;
b = 0.2;
c = 5.7;

%%% variables

x=v(1);
y=v(2);
z=v(3);
%%% equations
%dxdt = -y-z; dydt = x+a*y; dzdt = b + z*(x-c)
dv = [
    -y-z;  % dx/dt
    x + a*y;  	% dy/dt
    b + z*(x-c) %dz/dt
] ;
end
