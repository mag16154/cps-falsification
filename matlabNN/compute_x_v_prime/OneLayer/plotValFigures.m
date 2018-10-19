function [] = plotValFigures(sum_x_validation_norm, sum_v_validation_norm, time_steps, model_name)
figure(1);
clf;
xlabel('Time');
title(model_name);
total_output_points = 1:1:time_steps-1;
%legend('Output','Target');
plot(total_output_points(1, :), sum_x_validation_norm(1,:), total_output_points(1,:), sum_v_validation_norm(1,:));
end
