%% 2 dimensional nonlinear system with dynamics
%% x = A + x * x * y - B * x - x
%% y = B * x - x * x * y


clear all;
clc;

no_of_dims = 2
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

%inputDelays = 1:4:6;
%feedbackDelays = 1:4:6;
%hiddenSizes = [10,5];
%net.trainParam.epochs = 100;
%net.layers{1}.size = 15;
%net.layers{1}.transferFcn = 'tansig';
%net.layers{1}.initFcn = 'initnw';
%net.layers{1}.name = 'Hidden Layer 1';
%net.initFcn = 'initlay';
%net.performFcn = 'mse';
%net.trainFcn='trainbr'; %default = 'trainlm'
%net.divideFcn = 'dividerand';
inputDelays = 0:1;
feedbackDelays = 1:1;
hiddenLayerSize = 10;
net = narxnet(inputDelays,feedbackDelays,hiddenLayerSize);

%net = layrecnet(1:2,10);
[inputs,inputStates,layerStates,targets] = preparets(net,inputSeries,{},targetSeries);

net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

% Train the Network
[net,tr] = train(net,inputs,targets,inputStates,layerStates);

% Test the Network
outputs = net(inputs,inputStates,layerStates);
errors = gsubtract(targets,outputs);
performance = perform(net,targets,outputs)

output_mat = cell2mat(outputs);
target_mat = cell2mat(targets);
[dim1 dim2] = size(output_mat);
total_points = 1:1:dim2;
figure(1);
clf;
xlabel('Time');
title('Brusselator');
legend('Output','Target');
subplot(2,1,1);
plot(total_points(1,:), output_mat(1,:), total_points(1,:), target_mat(1,:))
subplot(2,1,2);
plot(total_points(1,:), output_mat(2,:), total_points(1,:), target_mat(2,:))
% View the Network
view(net)
wts = getwb(net);
[b, IW, LW] = separatewb(net, wts);
%weights = net.IW{1,1};
%bias = net.b{1};

	
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