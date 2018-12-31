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

nL = layers;
net = network;

net.numInputs = 1; % 1 input source
net.numLayers = nL;

net.inputs{1}.size = cell_dim1;

net.biasConnect = ones(nL, 1);

% net.inputConnect(i,j) represents the presence of an input weight 
% connection going to the ith layer from the jth input.
% Specify that all inputs are connected to layer 1.
% No inputs are connected to the other layers.
net.inputConnect = [1; zeros(nL - 1, 1)];

% net.layerConnect(i,j) represents the presence of a layer-weight 
% connection going to the ith layer from the jth layer.
net.layerConnect = zeros(nL, nL);
for i = 2:nL
    net.layerConnect(i, i - 1) = 1;
end

% net.outputConnect(i) indicates whether to connect ith layer's output to
% the outside.
net.outputConnect = [zeros(1, nL - 1), 1];

for i = 1:nL - 1
    net.layers{i}.size = nNeurons;
    net.layers{i}.transferFcn = 'tansig'; % 'tansig'; 'poslin';
    net.layers{i}.initFcn = 'initnw';
end

net.inputs{1}.processFcns = {'mapminmax', 'removeconstantrows'};

net.layers{nL}.size = 3;
net.layers{nL}.initFcn = 'initnw';
net.layers{nL}.transferFcn = 'logsig'; % 'logsig'; 'poslin';

% Set the divide function to dividerand (divide training data randomly).
net.divideFcn = 'dividerand';
net.adaptFcn = 'adaptwb';
net.performFcn = 'mse';
net.trainFcn = 'trainlm';
net.plotFcns = {'plotperform', 'plottrainstate', 'ploterrhist', ...
                'plotconfusion', 'plotroc'};

[Xs,Xi,Ai,Ts] = preparets(net,train_input,train_target);
[net tr] = train(net, Xs, Ts, Xi, Ai);
%[net, tr] = train(net,train_input, train_target, {}, {}, ew);
%[net, tr] = train(net,train_input,train_target,{},{});
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
