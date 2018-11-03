function [inputSeries, targetSeries] = createNNInput(traj_x, time_steps, no_of_dims, traj_combs, jump_step)

[no_of_samples, elems] = size(traj_combs);
time_instances_per_sample = floor(time_steps/jump_step);
no_of_inp_oup_pairs = no_of_samples*(time_instances_per_sample);
x_xp_vp_inputs = zeros(no_of_dims*3, no_of_inp_oup_pairs);
v_outputs = zeros(no_of_dims, no_of_inp_oup_pairs);
for idx = 1:no_of_samples
	trajs = traj_combs(idx, :);		
	central_traj = traj_x(:, :, trajs(1));
	second_traj = traj_x(:, :, trajs(2));
	for idy = 1:jump_step:time_steps-1
		x_val = central_traj(idy, :);
        	v_val = second_traj(idy,:) - x_val; 
        	t_val = idy+jump_step;
		%t_val = idy+time_step_idx;     	
		x_prime_val = central_traj(t_val, :);
        	v_prime_val = second_traj(t_val, :) - x_prime_val;
		%time_step = traj_t(idy, idx);
		x_xp_vp_inp_pair = [x_val x_prime_val v_prime_val];
        	x_xp_vp_inputs(:, (idx-1)*(time_instances_per_sample) + idy) = x_xp_vp_inp_pair;
		v_outputs(:, (idx-1)*(time_instances_per_sample) + idy) = v_val;
	end
end
inputSeries = con2seq(x_xp_vp_inputs);
targetSeries = con2seq(v_outputs);
end
