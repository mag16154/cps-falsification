%% 2 dimensional nonlinear system with dynamics
%% x = A + x * x * y - B * x - x
%% y = B * x - x * x * y
%% Computing the inverse of v_prime

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
neurons = 15;
[net, output_v_values, target_v_values] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons);
plotFigures(output_v_values, target_v_values, no_of_dims, 'Brusselator');
[o_layer_output_vals, v_vals] = validateNN(net, traj_x, time_steps, no_of_dims, traj_combs);
validation_norm_values = zeros(no_of_samples, time_steps-1); 
for idx = 1:no_of_samples
	for idy=1:(time_steps-1)
		validation_norm_values(idx,idy) = norm(o_layer_output_vals(idx,:,idy) - v_vals(idx,:,idy));
	end
end
	
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
