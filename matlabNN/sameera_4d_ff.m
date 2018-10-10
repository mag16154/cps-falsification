%% 3 dimensional nonlinear system with dynamics
%% dxdt = sigma*(y - x); dydt = x*(rho-z) - y; dzdt = x*y - beta*z;

clear all;
clc;

%no_of_dims = 4
%no_of_samples = 1
 
%init_state_array = rand(no_of_samples,no_of_dims)*10

data_read = csvread('NNtrial.csv');

input_data = data_read(:, 1:4);
output_data = data_read(:, 5:8);
[dim1 dim2] = size(input_data);
%x_vals = zeros(dim1, dim2);
%y_vals = zeros(dim1, dim2);
%for idx = 1:dim1
%	x_vals input_data(idx, :);
%	y_vals(:, idx) = output_data(idx, :);
%end

input_data_t = input_data'
[dim1 dim2] = size(input_data_t);
input_mins = 1:dim1;
input_maxs = 1:dim1;
for idx = 1:dim1
	myrow = input_data_t(idx,:);
	mymin = min(myrow);
	mymax = max(myrow);
	input_mins(idx) = mymin;
	input_maxs(idx) = mymax; 
	for idy = 1:dim2
		input_data_t(idx,idy) = (myrow(idy)-mymin)/(mymax-mymin);	
	end
end

output_data_t = output_data'
[dim1 dim2] = size(output_data_t);
output_mins = 1:dim1;
output_maxs = 1:dim1;
for idx = 1:dim1
	myrow = output_data_t(idx,:);
	mymin = min(myrow);
	mymax = max(myrow);
	output_mins(idx) = mymin;
	output_maxs(idx) = mymax; 
	for idy = 1:dim2
		output_data_t(idx,idy) = (myrow(idy)-mymin)/(mymax-mymin);	
	end
end


inputSeries = con2seq(input_data_t);
targetSeries = con2seq(output_data_t);

[cell_dim1 cell_dim2] = size(inputSeries);
%no_train_samples = (cell_dim2*3)/4;
no_train_samples = 45;
no_test_samples = cell_dim2 - no_train_samples;
train_input = inputSeries(1, 1:no_train_samples);
train_target = targetSeries(1, 1:no_train_samples);
test_input = inputSeries(1, no_train_samples+1:cell_dim2);
test_target = targetSeries(1, no_train_samples+1:cell_dim2);
net = feedforwardnet(6);
net.trainParam.epochs = 100;
net.layers{1}.transferFcn = 'logsig';
%net.layers{2}.transferFcn = 'logsig';
net.trainFcn='trainbr'; %default = 'trainlm'
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

[dim1 dim2] = size(output_mat);
for idx = 1:dim1
	mymin = output_mins(idx);
	mymax = output_maxs(idx);
	for idy = 1:dim2
		output_mat(idx,idy) = (output_mat(idx, idy)*(mymax - mymin))+mymin;
		target_mat(idx,idy) = (target_mat(idx, idy)*(mymax - mymin))+mymin;
	end
end
err = immse(output_mat, target_mat);
%[dim1 dim2] = size(output_mat);

total_points = 1:1:no_test_samples;

figure(1);
clf;
xlabel('Time');
title('Steam');
subplot(3,1,1);
plot(output_mat(1,:), output_mat(2,:), target_mat(1,:), target_mat(2,:))
subplot(3,1,2);
plot(output_mat(1,:), output_mat(3,:), target_mat(1,:), target_mat(3,:))
subplot(3,1,3);
plot(output_mat(2,:), output_mat(3,:), target_mat(2,:), target_mat(3,:))

figure(2);
subplot(4,1,1);
plot(total_points(1,:), output_mat(1,:), total_points(1,:), target_mat(1,:))
subplot(4,1,2);
plot(total_points(1,:), output_mat(2,:), total_points(1,:), target_mat(2,:))
subplot(4,1,3);
plot(total_points(1,:), output_mat(3,:), total_points(1,:), target_mat(3,:))
subplot(4,1,4);
plot(total_points(1,:), output_mat(4,:), total_points(1,:), target_mat(4,:))
