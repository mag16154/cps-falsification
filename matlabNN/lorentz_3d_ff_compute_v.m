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

[time_steps elements] = size(traj_t)
x_xp_vp_t_inputs = zeros(no_of_dims*3+1, (no_of_samples-1)*(time_steps-1))
v_outputs = zeros(no_of_dims, (no_of_samples-1)*(time_steps-1))
for idx = 1:no_of_samples
	trajs = traj_combs(idx, :);		
	central_traj = traj_x(:, :, trajs(1));
	second_traj = traj_x(:, :, trajs(2));	
	for idy = 1:(time_steps-1)
		x_val = central_traj(idy, :);
        	v_val = second_traj(idy,:) - x_val; 
        	time_step = traj_t(idy, idx);
        	x_prime_val = central_traj(idy+1, :);
        	v_prime_val = second_traj(idy+1, :) - x_prime_val;
		x_xp_vp_t_inp_pair = [x_val x_prime_val v_prime_val time_step];
        	x_xp_vp_t_inputs(:, (idx-1)*(time_steps-1) + idy) = x_xp_vp_t_inp_pair;
		v_outputs(:, (idx-1)*(time_steps-1) + idy) = v_val;
	end
end

inputSeries = con2seq(x_xp_vp_t_inputs);
targetSeries = con2seq(v_outputs);

[cell_dim1 cell_dim2] = size(inputSeries);
total_points = 1:1:cell_dim2;
total_points = total_points(randperm(length(total_points)));
no_train_samples = (cell_dim2*2)/3;
no_test_samples = cell_dim2 - no_train_samples;
train_input = cell(1, no_train_samples);
train_target = cell(1, no_train_samples);
test_input = cell(1, no_test_samples);
test_target = cell(1, no_test_samples);
for idx = 1:no_train_samples
    train_input(idx) = inputSeries(total_points(idx));
    train_target(idx) = targetSeries(total_points(idx));
end
for idx = 1:no_test_samples
    test_input(idx) = inputSeries(total_points(no_train_samples+idx));
    test_target(idx) = targetSeries(total_points(no_train_samples+idx));
end

net = feedforwardnet(15);
net.trainParam.epochs = 100;
[net tr] = train(net,train_input, train_target);

% View the Network
view(net)
wts = getwb(net);
[b, IW, LW] = separatewb(net, wts);
%weights = net.IW{1,1};
%bias = net.b{1};

outputs = net(test_input);
perf = perform(net,outputs,test_target)

output_mat = cell2mat(outputs);
target_mat = cell2mat(test_target);
output_v_values = output_mat(1:3,:);
target_v_values = target_mat(1:3,:);
[dim1 dim2] = size(output_mat);
total_output_points = 1:1:dim2;

pts_to_be_plotted = 1500
figure(1);
clf;
xlabel('Time');
title('Lorentz');
legend('Output','Target');
subplot(3,1,1);
plot(total_output_points(1,1:pts_to_be_plotted), output_v_values(1,1:pts_to_be_plotted), total_output_points(1,1:pts_to_be_plotted), target_v_values(1,1:pts_to_be_plotted));
subplot(3,1,2);
plot(total_output_points(1,1:pts_to_be_plotted), output_v_values(2,1:pts_to_be_plotted), total_output_points(1,1:pts_to_be_plotted), target_v_values(2,1:pts_to_be_plotted))
subplot(3,1,3);
plot(total_output_points(1,1:pts_to_be_plotted), output_v_values(3,1:pts_to_be_plotted), total_output_points(1,1:pts_to_be_plotted), target_v_values(3,1:pts_to_be_plotted))
	
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

