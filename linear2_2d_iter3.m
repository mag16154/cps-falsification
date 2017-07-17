
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

X = [8.5 9.5];
Y = [5.5 6.5];

%% Co-ordinates of an unsafe state
usafe = [4.8;2.4];

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

%% Compute the distance for the central trajectory
for idt = 1:size(traj_t,1);
	pair(1,:) = traj_x(idt,:,1);
	pair(2,:) = usafe(:);
	dist_center_usafe(idt,:) = pair(2,:) - pair(1,:);
	dist_center_usafe_euclid(idt) = pdist(pair,'euclidean');
end

point = [5.1839 2.1829; 5.1998 2.1839;];
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
	beta(idt,:) = inv_base(:,:)*transpose(dist_center_usafe(idt,:));
end

for idt = 1:size(traj_t,1);
	max_beta_vector(idt,1) = abs(beta(idt,1));
	if (max_beta_vector(idt,1) < abs(beta(idt,2)));
		max_beta_vector(idt,1) = abs(beta(idt,2));
	end
end

min_beta = max_beta_vector(1,1);
min_beta_time_step = 1;
for idt = 2:size(traj_t,1);
	for i = 1:1;
		if (min_beta > max_beta_vector(idt,i))
			min_beta = max_beta_vector(idt,i);
			min_beta_time_step = idt;
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
figure(2);
plot(traj_x(:,1,1),traj_x(:,2,1), traj_x(:,1,no_of_elements),traj_x(:,2,no_of_elements), usafe(1),usafe(2),'-o');
xlabel('X'),
ylabel('Y'),

%% Plots first, second and third trajectories, where y is changed in 2nd and x is changed in 3rd
figure(3);
plot(traj_x(:,1,1), traj_x(:,2,1), traj_x(:,1,3), traj_x(:,2,3), traj_x(:,1,2), traj_x(:,2,2), usafe(1), usafe(2), '-o');
xlabel('X'),
ylabel('Y'),
legend('init','Y','X'),

%% Plots max beta at each time step
figure(4);
plot(traj_t(:,1), max_beta_vector(:));
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


