function [inputSeries, targetSeries] = createNNInput(traj_x, time_steps, no_of_dims, traj_combs)

[no_of_samples, elems] = size(traj_combs);
x_v_t_inputs = zeros(no_of_dims*2+1, (no_of_samples-1)*(time_steps-1))
x_v_outputs = zeros(no_of_dims*2, (no_of_samples-1)*(time_steps-1))
for idx = 1:no_of_samples
	trajs = traj_combs(idx, :);		
	central_traj = traj_x(:, :, trajs(1));
	second_traj = traj_x(:, :, trajs(2));	
	for idy = 1:(time_steps-1)
		x_val = central_traj(idy, :);
        	v_val = second_traj(idy,:) - x_val; 
        	%time_step = traj_t(idy, idx);
		time_step_idx = randi(time_steps-idy);
		t_val = idy+time_step_idx;        	
		x_v_t_inp_pair = [x_val v_val t_val];
        	x_prime_val = central_traj(t_val, :);
        	v_prime_val = second_traj(t_val, :) - x_prime_val;
        	x_v_oup_pair = [x_prime_val, v_prime_val];
		x_v_t_inputs(:, (idx-1)*(time_steps-1) + idy) = x_v_t_inp_pair;
		x_v_outputs(:, (idx-1)*(time_steps-1) + idy) = x_v_oup_pair;
	end
end
inputSeries = con2seq(x_v_t_inputs);
targetSeries = con2seq(x_v_outputs);
end
