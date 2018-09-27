%% 3 dimensional nonlinear system with dynamics
%% dxdt = -y-z; dydt = x+a*y; dzdt = b + z*(x-c)

clear all;
clc;

no_of_dims = 3
no_of_samples = 20
 
init_state_array = rand(no_of_samples,no_of_dims)*10

%% Time

tspan = 0:0.01:10;

%% Integration/Simulation

for idx = 1:no_of_samples
%	localy0 = [Y0(idx,1) Y0(idx,2)];
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	traj_t(:,idx) = t;
	traj_x(:,:,idx) = x;
end

[time_steps elements] = size(traj_t)
x_t_inputs = zeros(no_of_dims+1, no_of_samples*(time_steps-1))
x_outputs = zeros(no_of_dims, no_of_samples*(time_steps-1))
for idx = 1:no_of_samples
	for idy = 1:(time_steps-1)
		x_val = traj_x(idy, :, idx);
		time_step = traj_t(idy, idx);
		x_t_pair = [x_val time_step];
		%abc=size(x_t_pair);
		%output = traj_x(idy+1, :, idx);
		x_t_inputs(:, (idx-1)*(time_steps-1) + idy) = x_t_pair;
		x_outputs(:, (idx-1)*(time_steps-1) + idy) = traj_x(idy+1, :, idx);
	end
end

%load ph_dataset
%inputSeries = phInputs;
%targetSeries = phTargets;

%abc = size(inputSeries);
%abc = inputSeries(:,1);
%abc = [0.2;0.2];
inputSeries = con2seq(x_t_inputs);
targetSeries = con2seq(x_outputs);

net = feedforwardnet(10);
net.trainParam.epochs = 100;
[net tr] = train(net,inputSeries,targetSeries);
view(net)
outputs = net(inputSeries);
perf = perform(net,outputs,targetSeries)

output_mat = cell2mat(outputs);
target_mat = cell2mat(targetSeries);
[dim1 dim2] = size(output_mat);
total_points = 1:1:dim2;
figure(1);
clf;
xlabel('Time');
title('Vanderpol');
subplot(2,1,1);
plot(total_points(1,:), output_mat(1,:), total_points(1,:), target_mat(1,:))
subplot(2,1,2);
plot(total_points(1,:), output_mat(2,:), total_points(1,:), target_mat(2,:))
% View the Network
view(net)
	
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
