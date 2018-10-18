function [] = plotFigures(output_v_values, target_v_values, no_of_dims, model_name)

[dim1 dim2] = size(output_v_values);
total_test_points = 1:1:dim2;
test_norm_values = zeros(1,dim2);
for idx=1:dim2
	test_norm_values(1,idx) = norm(output_v_values(:,idx)- target_v_values(:,idx));
end

pts_to_be_plotted = 1500

figure(1);
clf;
xlabel('Time');
title(model_name);
legend('Output','Target');
if no_of_dims == 2
	subplot(3,1,1);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(1,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(1,1:pts_to_be_plotted));
	subplot(3,1,2);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(2,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(2,1:pts_to_be_plotted));	
	subplot(3,1,3);
	plot(total_test_points(1,1:pts_to_be_plotted), test_norm_values(1,1:pts_to_be_plotted));
elseif no_of_dims == 3
	subplot(4,1,1);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(1,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(1,1:pts_to_be_plotted));
	subplot(4,1,2);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(2,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(2,1:pts_to_be_plotted));
	subplot(4,1,3);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(3,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(3,1:pts_to_be_plotted));
	subplot(4,1,4);
	plot(total_test_points(1,1:pts_to_be_plotted), test_norm_values(1,1:pts_to_be_plotted));
elseif no_of_dims == 4
	subplot(5,1,1);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(1,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(1,1:pts_to_be_plotted));
	subplot(5,1,2);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(2,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(2,1:pts_to_be_plotted));
	subplot(5,1,3);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(3,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(3,1:pts_to_be_plotted));
	subplot(5,1,4);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(4,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(4,1:pts_to_be_plotted));
	subplot(5,1,5);
	plot(total_test_points(1,1:pts_to_be_plotted), test_norm_values(1,1:pts_to_be_plotted));
elseif no_of_dims == 7
	subplot(5,1,1);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(1,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(1,1:pts_to_be_plotted));
	subplot(5,1,2);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(2,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(2,1:pts_to_be_plotted))
	subplot(5,1,3);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(3,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(3,1:pts_to_be_plotted))
	subplot(5,1,4);
	plot(total_test_points(1,1:pts_to_be_plotted), output_v_values(4,1:pts_to_be_plotted), total_test_points(1,1:pts_to_be_plotted), target_v_values(4,1:pts_to_be_plotted))
	subplot(5,1,5);
	plot(total_test_points(1,1:pts_to_be_plotted), test_norm_values(1,1:pts_to_be_plotted));
end
