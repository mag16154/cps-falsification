function [] = plotFigures(output_mat, target_mat, no_of_dims, model_name)

[dim1 dim2] = size(output_mat);
total_output_points = 1:1:dim2;
output_x_plus_v_values = output_mat(1:no_of_dims,:) + output_mat(no_of_dims+1:2*no_of_dims,:);
target_x_plus_v_values = target_mat(1:no_of_dims,:) + target_mat(no_of_dims+1:2*no_of_dims,:);

figure(1);
clf;
xlabel('Time');
title(model_name);
%legend('Output','Target');
if no_of_dims == 2
	subplot(2,1,1);
    plot(total_output_points(1, :), output_x_plus_v_values(1,:), total_output_points(1,:), target_x_plus_v_values(1,:));
    subplot(2,1,2);
    plot(total_output_points(1, :), output_x_plus_v_values(2,:), total_output_points(1,:), target_x_plus_v_values(2,:));
elseif no_of_dims == 3
	subplot(3,1,1);
    plot(total_output_points(1, :), output_x_plus_v_values(1,:), total_output_points(1,:), target_x_plus_v_values(1,:));
    subplot(3,1,2);
    plot(total_output_points(1, :), output_x_plus_v_values(2,:), total_output_points(1,:), target_x_plus_v_values(2,:));
    subplot(3,1,3);
    plot(total_output_points(1, :), output_x_plus_v_values(3,:), total_output_points(1,:), target_x_plus_v_values(3,:));
elseif no_of_dims == 4
	subplot(4,1,1);
    plot(total_output_points(1, :), output_x_plus_v_values(1,:), total_output_points(1,:), target_x_plus_v_values(1,:));
    subplot(4,1,2);
    plot(total_output_points(1, :), output_x_plus_v_values(2,:), total_output_points(1,:), target_x_plus_v_values(2,:));
    subplot(4,1,3);
    plot(total_output_points(1, :), output_x_plus_v_values(3,:), total_output_points(1,:), target_x_plus_v_values(3,:));
    subplot(4,1,3);
    plot(total_output_points(1, :), output_x_plus_v_values(4,:), total_output_points(1,:), target_x_plus_v_values(4,:));
end
