function [net, output_mat, target_mat] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons, layers, preprocess)

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

if layers == 1
    net = feedforwardnet(neurons(1, 1));
elseif layers == 2
    net = feedforwardnet([neurons(1, 1), neurons(1, 2)]);
elseif layers == 3
    net = feedforwardnet([neurons(1, 1), neurons(1, 2), neurons(1, 3)]);
end

%net.trainFcn = 'trainbfg';
%net.performFcn = 'mae';
net.trainParam.epochs = epochs;
if preprocess == false
	net.inputs{1}.processFcns = {};
	if layers ==1 
		net.outputs{2}.processFcns = {};
	elseif layers == 2
		net.outputs{3}.processfcns = {};
	elseif layers == 3
		net.outputs{4}.processfcns = {};
	end
end

%[net, tr] = train(net,train_input, train_target);
[net, tr] = train(net,train_input,train_target,{},{},1./cell2mat(train_target));
%plotperform(tr);	
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
end
