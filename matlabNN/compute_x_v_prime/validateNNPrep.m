function [o_layer_output_vals, x_v_prime_vals] = validateNNPrep(net, traj_x, time_steps, no_of_dims, traj_combs) 

wts = getwb(net);
[b, IW, LW] = separatewb(net, wts);
[no_of_samples, elems] = size(traj_combs);
o_layer_output_vals = zeros(no_of_samples, 2*no_of_dims, time_steps-1);
x_v_prime_vals = zeros(no_of_samples, 2*no_of_dims, time_steps-1);
abc = net.inputs{1}.processFcns;
abc_3 = net.outputs{3}.processFcns;
for idx = 1:3
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
		for iii = 1:numel(net.inputs{1}.processFcns)
			h1_layer_input = feval( net.inputs{1}.processFcns{iii}, ...
			'apply', h1_layer_input, net.inputs{1}.processSettings{iii} );
		end
		IW_times_h1_input = mtimes(IW{1,1}, h1_layer_input);
		IW_times_h1_input_plus_bias = IW_times_h1_input + b{1,1};
		h1_layer_output = tansig(IW_times_h1_input_plus_bias);

		h2_layer_input = h1_layer_output;
		IW_times_h2_input = mtimes(LW{2,1}, h2_layer_input);
		IW_times_h2_input_plus_bias = IW_times_h2_input + b{2,1};
		h2_layer_output = tansig(IW_times_h2_input_plus_bias);
		
		IW_times_o_input = mtimes(LW{3,2}, h2_layer_output);
		IW_times_o_input_plus_bias = IW_times_o_input + b{3,1};
		%o_layer_output_vals(idx, :, idy) = purelin(IW_times_o_input_plus_bias);
		x_v_prime_vals(idx, :, idy) = [x_prime_val v_prime_val];
		o_layer_output = purelin(IW_times_o_input_plus_bias);
		for iii = 1:numel(net.outputs{3}.processFcns)
     			o_layer_output = feval( net.outputs{3}.processFcns{iii}, ...
         		'reverse', o_layer_output, net.outputs{3}.processSettings{iii} );
		end
		o_layer_output_vals(idx, :, idy) = o_layer_output(:);
%		validation_norm_values(idx,idy) = norm(o_layer_output(:)- v_val(:));
	end
end
end
