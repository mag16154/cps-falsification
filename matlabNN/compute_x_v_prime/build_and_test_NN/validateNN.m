function [v_mae] = validateNN(net, traj_x, time_steps, no_of_dims, traj_combs, layers, preprocess) 

wts = getwb(net);
[b, IW, LW] = separatewb(net, wts);
[no_of_samples, elems] = size(traj_combs);
o_layer_output_vals = zeros(no_of_samples, 2*no_of_dims, time_steps-1);
xprime_vprime_vals = zeros(no_of_samples, 2*no_of_dims, time_steps-1);
v_mae = zeros(no_of_samples, time_steps-1);

if layers == 1
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
			h1_layer_output = logsig(IW_times_h1_input_plus_bias);
		
			IW_times_o_input = mtimes(LW{2,1}, h1_layer_output);
			IW_times_o_input_plus_bias = IW_times_o_input + b{2,1};
			xprime_vprime_vals(idx, :, idy) = [x_prime_val v_prime_val];
			o_layer_output = purelin(IW_times_o_input_plus_bias);
			o_layer_output_vals(idx, :, idy) = o_layer_output(:);
			v_mae(idx, idy) = mae(o_layer_output_vals(idx, :, idy) - xprime_vprime_vals(idx, :, idy));
		end
	end
elseif layers == 2
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
			h1_layer_output = logsig(IW_times_h1_input_plus_bias);

			h2_layer_input = h1_layer_output;
			IW_times_h2_input = mtimes(LW{2,1}, h2_layer_input);
			IW_times_h2_input_plus_bias = IW_times_h2_input + b{2,1};
			h2_layer_output = logsig(IW_times_h2_input_plus_bias);
		
			IW_times_o_input = mtimes(LW{3,2}, h2_layer_output);
			IW_times_o_input_plus_bias = IW_times_o_input + b{3,1};
			xprime_vprime_vals(idx, :, idy) = [x_prime_val v_prime_val];
			o_layer_output = purelin(IW_times_o_input_plus_bias);
			o_layer_output_vals(idx, :, idy) = o_layer_output(:);
			v_mae(idx, idy) = mae(o_layer_output_vals(idx, :, idy) - xprime_vprime_vals(idx, :, idy));
		end
	end
elseif layers == 3
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
			h1_layer_output = logsig(IW_times_h1_input_plus_bias);

			h2_layer_input = h1_layer_output;
			IW_times_h2_input = mtimes(LW{2,1}, h2_layer_input);
			IW_times_h2_input_plus_bias = IW_times_h2_input + b{2,1};
			h2_layer_output = tansig(IW_times_h2_input_plus_bias);

			h3_layer_input = h2_layer_output;
			IW_times_h3_input = mtimes(LW{3,2}, h3_layer_input);
			IW_times_h3_input_plus_bias = IW_times_h3_input + b{3,1};
			h3_layer_output = tansig(IW_times_h3_input_plus_bias);
		
			IW_times_o_input = mtimes(LW{4,3}, h3_layer_output);
			IW_times_o_input_plus_bias = IW_times_o_input + b{4,1};
			xprime_vprime_vals(idx, :, idy) = [x_prime_val v_prime_val];
			o_layer_output = purelin(IW_times_o_input_plus_bias);
			o_layer_output_vals(idx, :, idy) = o_layer_output(:);
			v_mae(idx, idy) = mae(o_layer_output_vals(idx, :, idy) - xprime_vprime_vals(idx, :, idy));
		end
	end
end
v_mae = mse(o_layer_output_vals(:, 1:no_of_dims, :), xprime_vprime_vals(:, 1:no_of_dims, :));
end
