
%% 2 dimensional linear system with dynamics
%% dx = -x, dy = x-y

%% Apply the superposition principle to represent 
%% the distance vector to the unsafe state as a linear
%% combination of the basis vectors.

clear all;
clc;

%% Initial conditions
%% Values later will be designated as 
%% x1, x2, x3, y1, y2, y3.

X = [5.6375 9];
Y = [2.706 6];

%% Co-ordinates of an unsafe states
%% first is the center, second is along Y axis
%% and third is along x axis.
usafe = [5 3; 5 3.5; 5.5 3; 5.5 3.5];
%% Total number of initial states to be simulated
no_of_elements=numel(X)*numel(Y);

%% Populating the array of initial states
%% array_index -> state
%% 1 -> (x1,y1) -- init/center state
%% 2 -> (x1,y2)
%% 3 -> (x2,y1)
%% 4 -> (x2,y2)

init_state_array=ones(no_of_elements,2);
for myx=1:numel(X)
	for myy=1:numel(Y)
		x_idx=numel(Y)*(myx-1)+myy;
		init_state_array(x_idx,1) = X(myx);
		init_state_array(x_idx,2) = Y(myy);
	end
end

%% Time

tspan = 0:0.02:5;

%% Integration/Simulation

for idx = 1:no_of_elements
%	localy0 = [Y0(idx,1) Y0(idx,2)];
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	traj_t(:,idx) = t;
	%% First dim is time, 2nd is for x and y, 
	%% and 3rd is for # of initial states
	traj_x(:,:,idx) = x; 
end

if (exist('dist.mat') == 2);
	loaded_dist = load('dist.mat', 'dist_init_usafe_euclid');
end
%% Compute the distance for the initial trajectory
for i = [1 2 3 4];
	for idt = 1:size(traj_t,1);
		pair(1,:) = traj_x(idt,:,1);
		pair(2,:) = usafe(i,:);
		dist_init_usafe(idt,:,i) = pair(2,:) - pair(1,:);
		dist_init_usafe_euclid(idt,i) = pdist(pair,'euclidean');
	end
end

for i = [1 2 3 4];
	min_dist(i) = dist_init_usafe_euclid(1,i);
	min_dist_time_step(i) = 1;
	for idt = 2:size(traj_t,1);
		if(min_dist(i) > dist_init_usafe_euclid(idt,i));
			min_dist(i) = dist_init_usafe_euclid(idt,i);
			min_dist_time_step(i) = idt;
		end 
	end
end

save('dist.mat', 'dist_init_usafe_euclid');

point = [4.9844 1.9844; 4.9844 2.6414;];
dist_abc = pdist(point, 'euclidean');
%% Compute the base vectors for the trajectory@3 where x is changed 
%% and y is unchanged.
%% Compute the base_vectors for the trajectory@2 where x is unchanged 
%% and y is changed.

idy = [1 3 2];
for idt = 1:size(traj_t,1);
		i = 1;
		pair(1,:) = traj_x(idt,:,idy(i));
		i = i+1;
		pair(2,:) = traj_x(idt,:,idy(i));
		base_vectors(idt,:,1) = pair(2,:) - pair(1,:);
		i = i+1;
		pair(2,:) = traj_x(idt,:,idy(i));
		base_vectors(idt,:,2) = pair(2,:) - pair(1,:);
end


for idt = 1:size(traj_t,1);
	base_vector_at_idt(:,1) = base_vectors(idt,:,1);
	base_vector_at_idt(:,2) = base_vectors(idt,:,2);
	inv_base(:,:) = inv(base_vector_at_idt(:,:));
	beta_1(idt,:) = (inv_base(:,:)*transpose(dist_init_usafe(idt,:,1)));
	beta_2(idt,:) = (inv_base(:,:)*transpose(dist_init_usafe(idt,:,2)));
	beta_3(idt,:) = (inv_base(:,:)*transpose(dist_init_usafe(idt,:,3)));
end

for idt = 1:size(traj_t,1);
	max_beta_vector(1,idt) = abs(beta_1(idt,1));
	if (max_beta_vector(1,idt) < abs(beta_1(idt,2)));
		max_beta_vector(1,idt) = abs(beta_1(idt,2));
	end
	max_beta_vector(2,idt) = abs(beta_2(idt,1));
	if (max_beta_vector(2,idt) < abs(beta_2(idt,2)));
		max_beta_vector(2,idt) = abs(beta_2(idt,2));
	end
	max_beta_vector(3,idt) = abs(beta_3(idt,1));
	if (max_beta_vector(3,idt) < abs(beta_3(idt,2)));
		max_beta_vector(3,idt) = abs(beta_3(idt,2));
	end
end

if (exist('beta.mat') == 2)
	loaded_max_beta_vector = load('beta.mat', 'max_beta_vector');
	for i = [1 2 3];
		for idt = 1:size(traj_t,1);
			if (loaded_max_beta_vector.max_beta_vector(i,idt) < max_beta_vector(i,idt))
				discrepancy(i,idt) = 1;
			else
				discrepancy(i,idt) = 0;
			end	
		end
	end
end
save('beta', 'max_beta_vector');

for i = [1 2 3];
	min_beta(i) = max_beta_vector(i,1);
	min_beta_time_step(i) = 1;
	for idt = 2:size(traj_t,1);
		if (min_beta(i) > max_beta_vector(i,idt))
			min_beta(i) = max_beta_vector(i,idt);
			min_beta_time_step(i) = idt;
		end
	end
end

%% Compute the distance for the trajectory@9 where both x and y 
%% are changed.

%for idx = 1:size(traj_t,1);
%	for i = 1:1;
%	pair(1,:) = traj_x(idx,:,no_of_elements);
%		pair(2,:) = usafe(:);
%		dist_x_y_usafe(idx,i) = pdist(pair,'euclidean');
%	end
%end

%% Plot the graphs

figure(1);
clf;
%% Plots x and y w.r.t. time
plot(traj_t(:,1), traj_x(:,:,1), traj_t(:,no_of_elements), traj_x(:,:,no_of_elements));
xlabel('Time'),
title('Linear System 2');
legend('X','Y');

%% Plots first and the last trajectories
%figure(2);
%plot(traj_x(:,1,1),traj_x(:,2,1), traj_x(:,1,no_of_elements),traj_x(:,2,no_of_elements), usafe(1,1), usafe(1,2), '-o', usafe(2,1), usafe(2,2), '-o', usafe(3,1), usafe(3,2),'-o');
%xlabel('X'),
%ylabel('Y'),

%% Plots first, second and third trajectories, where y is changed in 2nd and x is changed in 3rd
figure(3);
plot(traj_x(:,1,1), traj_x(:,2,1), traj_x(:,1,3), traj_x(:,2,3), traj_x(:,1,2), traj_x(:,2,2), usafe(1,1), usafe(1,2), '-o');
xlabel('X'),
ylabel('Y'),
legend('init','Y','X'),

%% Plots max beta at each time step
figure(4);
plot(traj_t(:,1), max_beta_vector(:,:));
xlabel('Time'),
ylabel('Beta'),
% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%%% parameter set 1

k1=1;
k2=1;

%%% variables

x=v(1);
y=v(2);
%%% equations

dv = [
    -k1*x;    	% dx/dt
    k1*x-k2*y;  % dy/dt
] ;
end


