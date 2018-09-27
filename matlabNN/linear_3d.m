%% 3 dimensional linear system with dynamics
%% dx = -x, dy = x-y, dz = y

clear all;
clc;

%% Initial conditions
%% Values later will be designated as 
%% x1, x2, x3, y1, y2, y3, z1, z2, z3.

%% Assuming initial set is [0,4]x[0,3]x[0,3]
%% We start from the state (0,0,0).

X = 2.0:0.5:3.0;
Y = 0:0.5:1.0;
Z = 0:0.5:1.0;

no_of_elements = 10
 
init_state_array = rand(no_of_elements,3)*2 + 3

%% Time

tspan = 0:0.02:10;

%% Integration/Simulation

for idx = 1:no_of_elements
%	localy0 = [Y0(idx,1) Y0(idx,2)];
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	traj_t(:,idx) = t;
	traj_x(:,:,idx) = x;
end

[time_steps elements] = size(traj_t)
x_t_inputs = zeros(4, no_of_elements*(time_steps-1))
x_outputs = zeros(3, no_of_elements*(time_steps-1))
for idx = 1:no_of_elements
	for idy = 1:(time_steps-1)
		x_val = traj_x(idy, :, idx);
		time_step = traj_t(idy, idx);
		x_t_pair = [x_val time_step];
		abc=size(x_t_pair);
		output = traj_x(idy+1, :, idx);
		x_t_inputs(:, (idx-1)*time_steps + idy) = x_t_pair;
		x_outputs(:, (idx-1)*time_steps + idy) = output;
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

inputDelays = 1:4;
feedbackDelays = 1:4;
hiddenLayerSize = 10;
net = narxnet(inputDelays,feedbackDelays,hiddenLayerSize);

%net = layrecnet(1:2,10);
[inputs,inputStates,layerStates,targets] = ... 
   	preparets(net,inputSeries,{},targetSeries);

net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;

% Train the Network
[net,tr] = train(net,inputs,targets,inputStates,layerStates);

% Test the Network
outputs = net(inputs,inputStates,layerStates);
errors = gsubtract(targets,outputs);
performance = perform(net,targets,outputs)

% View the Network
view(net)
	
% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set 1

k1=1;
k2=1;

%%% variables

x=v(1);
y=v(2);
z=v(3);
%%% equations

dv = [
    -k1*x;    	% dx/dt
    k1*x-k2*y;  % dy/dt
    k2*y;	% dz/dt
] ;
end