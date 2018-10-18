%% 2 dimensional nonlinear system with dynamics
%% x = y
%% y = mu*(1-x*x)*y - x
%% Computing basis vectors for each pair of trajectories


clear all;
clc;

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
[inputSeries, targetSeries] = createNNInput(traj_x, time_steps, no_of_dims, traj_combs);
epochs = 100;
neurons = 30;
[net, output_mat, target_mat] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons);
[o_layer_output_vals, x_v_prime_vals] = validateNNPrep(net, traj_x, time_steps, no_of_dims, traj_combs);
validation_x_norm_values = zeros(no_of_samples, time_steps-1);
validation_v_norm_values = zeros(no_of_samples, time_steps-1);
for idx = 1:no_of_samples
	for idy=1:(time_steps-1)
		validation_x_norm_values(idx,idy) = norm(o_layer_output_vals(idx,1:no_of_dims,idy) - x_v_prime_vals(idx,1:no_of_dims,idy));
		validation_v_norm_values(idx,idy) = norm(o_layer_output_vals(idx,no_of_dims+1:2*no_of_dims,idy) - x_v_prime_vals(idx,no_of_dims+1:2*no_of_dims,idy));
	end
end

plotFigures(output_mat, target_mat, no_of_dims, 'Vanderpol');
	
% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set 1

mu=1;

%%% variables

x=v(1);
y=v(2);
%%% equations

dv = [
    y;  % dx/dt
    mu*(1-x*x)*y - x;  	% dy/dt
] ;
end
