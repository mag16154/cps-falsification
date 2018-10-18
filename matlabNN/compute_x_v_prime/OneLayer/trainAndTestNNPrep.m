function [net, output_mat, target_mat] = trainAndTestNN(inputSeries, targetSeries, epochs, neurons)

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

net = feedforwardnet(neurons);
net.trainParam.epochs = epochs;
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
end
