%% 3 dimensional nonlinear system with dynamics
%% dxdt = sigma*(y - x); dydt = x*(rho-z) - y; dzdt = x*y - beta*z;
%% Computing basis vectors for each pair of trajectories

clear all;
clc;

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
epochs = 100;
neurons = 15;
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


[dim1 dim2] = size(output_mat);
total_output_points = 1:1:dim2;
output_x_plus_v_values = output_mat(1:no_of_dims,:) + output_mat(no_of_dims+1:2*no_of_dims,:);
target_x_plus_v_values = target_mat(1:no_of_dims,:) + target_mat(no_of_dims+1:2*no_of_dims,:);
figure(1);
clf;
xlabel('Time');
title('Lorentz');
legend('Output','Target');
subplot(3,1,1);
plot(total_output_points(1, :), output_x_plus_v_values(1,:), total_output_points(1,:), target_x_plus_v_values(1,:));
subplot(3,1,2);
plot(total_output_points(1, :), output_x_plus_v_values(2,:), total_output_points(1,:), target_x_plus_v_values(2,:));
subplot(3,1,3);
plot(total_output_points(1, :), output_x_plus_v_values(3,:), total_output_points(1,:), target_x_plus_v_values(3,:));
	
% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set

sigma = 10.0;
rho = 28.0;
beta = 8.0 / 3.0;

%%% variables

x=v(1);
y=v(2);
z=v(3);
%%% equations
%dxdt = sigma*(y - x); dydt = x*(rho-z) - y; dzdt = x*y - beta*z;
dv = [
    sigma*(y - x);  % dx/dt
    x*(rho-z) - y;  % dy/dt
    x*y - beta*z; %dz/dt
] ;
end

