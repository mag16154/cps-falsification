function [x_mse, v_mse] = validateNN(net, traj_x, time_steps, no_of_dims, traj_combs) 

wts = getwb(net);
[b, IW, LW] = separatewb(net, wts);
[no_of_samples, elems] = size(traj_combs);
o_layer_output_vals = zeros(no_of_samples, 2*no_of_dims, time_steps-1);
x_v_prime_vals = zeros(no_of_samples, 2*no_of_dims, time_steps-1);
for idx = 1:no_of_samples
	trajs = traj_combs(idx, :);
	central_traj = traj_x(:, :, trajs(1));
	second_traj = traj_x(:, :, trajs(2));
	for idy=1:(time_steps-1)
		x_val = central_traj(idy, :);
        	v_val = second_traj(idy,:) - x_val;
		time_step_idx = randi(time_steps-idy);
		t_val = idy+time_step_idx;       
        	x_prime_val = central_traj(t_val, :);
        	v_prime_val = second_traj(t_val, :) - x_prime_val;
		h1_layer_input = [x_val v_val t_val]';
		IW_times_h1_input = mtimes(IW{1,1}, h1_layer_input);
		IW_times_h1_input_plus_bias = IW_times_h1_input + b{1,1};
		h1_layer_output = tansig(IW_times_h1_input_plus_bias);
		
		IW_times_o_input = mtimes(LW{2,1}, h1_layer_output);
		IW_times_o_input_plus_bias = IW_times_o_input + b{2,1};
		%o_layer_output_vals(idx, :, idy) = purelin(IW_times_o_input_plus_bias);
		x_v_prime_vals(idx, :, idy) = [x_prime_val v_prime_val];
		o_layer_output = purelin(IW_times_o_input_plus_bias);
		o_layer_output_vals(idx, :, idy) = o_layer_output(:);
	end
end

%validation_x_norm_values = zeros(no_of_samples, time_steps-1);
%validation_v_norm_values = zeros(no_of_samples, time_steps-1);
x_mse = immse(o_layer_output_vals(:, 1:no_of_dims, :), x_v_prime_vals(:, 1:no_of_dims, :));
v_mse = immse(o_layer_output_vals(:, no_of_dims+1:2*no_of_dims, :), x_v_prime_vals(:, no_of_dims+1:2*no_of_dims, :));
%for idx = 1:no_of_samples
%	for idy=1:(time_steps-1)
%		validation_x_norm_values(idx,idy) = norm(o_layer_output_vals(idx,1:no_of_dims,idy) - x_v_prime_vals(idx,1:no_of_dims,idy));
%		validation_v_norm_values(idx,idy) = norm(o_layer_output_vals(idx,no_of_dims+1:2*no_of_dims,idy) - x_v_prime_vals(idx,no_of_dims+1:2*no_of_dims,idy));
%	end
%end

%sum_x_validation_norm = zeros(1,time_steps-1);
%sum_v_validation_norm = zeros(1,time_steps-1);
%for idy = 1:(time_steps-1)
%	sum_x_validation_norm(idy) = sum(validation_x_norm_values(:,idy))/no_of_samples;
%	sum_v_validation_norm(idy) = sum(validation_v_norm_values(:,idy))/no_of_samples;
%end
end
