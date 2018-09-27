%% 3 dimensional linear system with dynamics
%% dx = -x, dy = x-y, dz = y

%% Apply the superposition principle to represent 
%% the distance vector to the unsafe state as a linear
%% combination of the basis vectors.

clear all;
clc;

%% Initial conditions
%% Values later will be designated as 
%% x1, x2, y1, y2, z1, z2.

X = [4.0204 5];
Y = [2.1969 3];
Z = [2.2824 3];

%% Co-ordinates of an unsafe state
%usafe = [1.3;0.6;1.08];
usafe = [3.1; 2.5; 2.9];
%% Total number of initial states to be simulated
no_of_elements=numel(X)*numel(Y)*numel(Z);

%% Populating the array of initial states
%% array_index -> state
%% 1 -> (x1,y1,z1) -- init/center state
%% 2 -> (x1,y1,z2) -- z changed
%% 3 -> (x1,y2,z1) -- y changed
%% 4 -> (x1,y2,z2)
%% 5 -> (x2,y1,z1) -- x changed
%% 6 -> (x2,y1,z2)
%% 7 -> (x2,y2,z1)
%% 8 -> (x2,y2,z2)
init_state_array=ones(no_of_elements,3);
for myx=1:numel(X)
	for myy=1:numel(Y)
		for myz=1:numel(Z)
			myidx=numel(Z)*numel(Y)*(myx-1)+numel(Z)*(myy-1)+myz;
			init_state_array(myidx,1) = X(myx);
			init_state_array(myidx,2) = Y(myy);
			init_state_array(myidx,3) = Z(myz);
		end
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
	%% First dim is time, 2nd is for x,y and z, 
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

%% Just to compute the distance between points
%% considering max_beta and actual_beta
point = [4.0204 2.1969 2.2824;
	4.0432 2.2824 2.2824;];
dist_abc = pdist(point, 'euclidean');
%% Compute the base vectors for the trajectory@5 where x is changed 
%% and y,z are unchanged.
%% Compute the base_vectors for the trajectory@3 where y is changed 
%% and x,z are unchanged.
%% Compute the base_vectors for the trajectory@2 where z is changed
%% and x,y are unchanged.

idy = [1 5 3 2];
for idt = 1:size(traj_t,1);
	i = 1;
	pair(1,:) = traj_x(idt,:,i);
	i = i+1;
	pair(2,:) = traj_x(idt,:,idy(i));
	base_vectors(idt,:,1) = pair(2,:) - pair(1,:);
	i = i+1;
	pair(2,:) = traj_x(idt,:,idy(i));
	base_vectors(idt,:,2) = pair(2,:) - pair(1,:);
	i = i+1;
	pair(2,:) = traj_x(idt,:,idy(i));
	base_vectors(idt,:,3) = pair(2,:) - pair(1,:);
end

for idt = 1:size(traj_t,1);
	for i = 1:3;
		base_vector_at_idt(:,i) = base_vectors(idt,:,i);
	end
	inv_base(:,:) = inv(base_vector_at_idt(:,:));
	beta(idt,:) = inv_base(:,:)*transpose(dist_center_usafe(idt,:));
end

for idt = 1:size(traj_t,1);
	max_beta_vector(idt) = abs(beta(idt,1));
	if (max_beta_vector(idt) < abs(beta(idt,2)));
		max_beta_vector(idt) = abs(beta(idt,2));
	end
	if (max_beta_vector(idt) < abs(beta(idt,3)));
		max_beta_vector(idt) = abs(beta(idt,3));
	end
end

min_beta = max_beta_vector(1,1);
min_beta_time_step = 1;
for idt = 2:size(traj_t,1);
	if (min_beta > max_beta_vector(idt))
		min_beta = max_beta_vector(idt);
		min_beta_time_step = idt;
	end
end

%% Plot the graphs

figure(1);
clf;
%% Plots x,y and z w.r.t. time
plot(traj_t(:,1), traj_x(:,:,1), traj_t(:,no_of_elements), traj_x(:,:,no_of_elements));
xlabel('Time'),
title('Linear System 2'),
legend('X','Y','Z'),

%% Plots max beta at each time step
figure(2);
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
z=v(3);
%%% equations

dv = [
    -k1*x;    	% dx/dt
    k1*x-k2*y;  % dy/dt
    k2*y;	% dz/dt	
] ;
end

