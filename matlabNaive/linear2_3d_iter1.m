
%% 3 dimensional linear system with dynamics
%% dx = -x, dy = x-y, dz = y

%% Consider just the distance from the unsafe state
%% Compute the number of times a simulation is closer 
%% to the usafe state along each dimension.

clear all;
clc;

%% Initial conditions
%% Values later will be designated as 
%% x1, x2, x3, y1, y2, y3, z1, z2, z3.

%% Assuming initial set is [0,4]x[0,3]x[0,3]
%% We start from the state (0,0,0).

%X = 2.0:0.5:3.0;
%Y = 0:0.5:1.0;
%Z = 0:0.5:1.0;
%usafe = [1.3;0.6;1.08];
X = 4:0.5:5;
Y = 2:0.5:3;
Z = 2:0.5:3;
%% Co-ordinates of an unsafe state
usafe = [3.1;2.5;2.9];

%% Total number of initial states to be simulated
no_of_elements=numel(X)*numel(Y)*numel(Z);

%% Populating the array of initial states
%% array_index -> state
%% 1 -> (x1,y1,z1) -- init/center state
%% 2 -> (x1,y1,z2)
%% 3 -> (x1,y1,z3) -- x and y unchanged, z changed
%% ....
%% 6 -> (x1,y2,z3)
%% 7 -> (x1,y3,z1) -- x and z unchanged, y changed
%% ....
%% 18 -> (x2,y3,z3)
%% 19 -> (x3,y1,z1) -- y and z unchanged, z changed
%% 20 -> (x3,y1,z2)
%% ....
%% 27 -> (x3,y3,z3)

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

tspan = 0:0.02:10;

%% Integration/Simulation

for idx = 1:no_of_elements
%	localy0 = [Y0(idx,1) Y0(idx,2)];
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	traj_t(:,idx) = t;
	traj_x(:,:,idx) = x;
end

%% Compute the distance for the central trajectory
for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,1);
		pair(2,:) = usafe(:);
		dist_center_usafe(idx,i) = pdist(pair,'euclidean');
	end
end

%% Compute the distance for the trajectory@19 where x is changed 
%% and z, y are unchanged.
%% Also compute the base vectors, just in case.

idy = [1 19];
for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,idy(i+1));
		pair(2,:) = traj_x(idx,:,idy(1));
		base_vectors_x(idx,i) = pdist(pair,'euclidean');
		pair(2,:) = usafe(:);
		dist_x_usafe(idx,i) = pdist(pair,'euclidean');
	end
end

%% Compute the distance for the trajectory@7 where x,z are unchanged 
%% and y is changed.
%% Also compute the base vectors, just in case.

idy = [1 7];
for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,idy(i+1));
		pair(2,:) = traj_x(idx,:,idy(1));
		base_vectors_y(idx,i) = pdist(pair,'euclidean');
		pair(2,:) = usafe(:);
		dist_y_usafe(idx,i) = pdist(pair,'euclidean');
	end
end	

%% Compute the distance for the trajectory@3 where x,y are unchanged 
%% and z is changed.
%% Also compute the base vectors, just in case.

idy = [1 3];
for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,idy(i+1));
		pair(2,:) = traj_x(idx,:,idy(1));
		base_vectors_z(idx,i) = pdist(pair,'euclidean');
		pair(2,:) = usafe(:);
		dist_z_usafe(idx,i) = pdist(pair,'euclidean');
	end
end	

%% Compute the distance for the trajectory@27 where all x, y
%% and z are changed.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,no_of_elements);
		pair(2,:) = usafe(:);
		dist_x_y_z_usafe(idx,i) = pdist(pair,'euclidean');
	end
end

%% Keep a track of the number of times the trajectory by changing x is 
%% perform than that the one from the central state.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		if (dist_center_usafe(idx,i) < dist_x_usafe(idx,i))
			pick_x(idx,i) = 0;
		else
			pick_x(idx,i) = 1;
		end
	end
end

%% Keep a track of the number of times the trajectory by changing x is 
%% perform than that the one from the central state.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		if (dist_center_usafe(idx,i) < dist_y_usafe(idx,i))
			pick_y(idx,i) = 0;
		else
			pick_y(idx,i) = 1;
		end
	end
end

%% Keep a track of the number of times the trajectory by changing z is 
%% perform than that the one from the central state.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		if (dist_center_usafe(idx,i) < dist_z_usafe(idx,i))
			pick_z(idx,i) = 0;
		else
			pick_z(idx,i) = 1;
		end
	end
end

%% Keep a track of the number of times the trajectory by changing x and y 
%% is perform than that the one from the central state.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		if (dist_center_usafe(idx,i) < dist_x_y_z_usafe(idx,i))
			pick_x_y_z(idx,i) = 0;
		else
			pick_x_y_z(idx,i) = 1;
		end
	end
end

%% Count the numbers along each direction

y_dim = 0;
x_dim = 0;
z_dim = 0;
x_y_z_dim = 0;
for idx = i:size(traj_t,1);
	if (pick_x(idx,1) == 1)
		x_dim = x_dim+1;
	end
	if (pick_y(idx,1) == 1)
		y_dim = y_dim+1;	
	end
	if (pick_z(idx,1) == 1)
		z_dim = z_dim+1;	
	end
	if (pick_x_y_z(idx,i) == 1)
		x_y_z_dim = x_y_z_dim+1;
	end
end

%% Find the minimum along each direction

min_x = dist_x_usafe(idx,1);
min_y = dist_y_usafe(idx,1);
min_z = dist_z_usafe(idx,1);
min_x_y_z = dist_x_y_z_usafe(idx,1);
min_init = dist_center_usafe(idx,1);
for idx = 2:size(traj_t,1);
	if (min_x > dist_x_usafe(idx,1))
		min_x = dist_x_usafe(idx,1);
	end
	if (min_y > dist_y_usafe(idx,1))
		min_y = dist_y_usafe(idx,1);
	end
	if (min_z > dist_z_usafe(idx,1))
		min_z = dist_z_usafe(idx,1);
	end
	if (min_x_y_z > dist_x_y_z_usafe(idx,1))
		min_x_y_z = dist_x_y_z_usafe(idx,1);
	end
	if (min_init > dist_center_usafe(idx,1))
		min_init = dist_center_usafe(idx,1);
	end
end

%% Compute the rate

rate_x = min_x/x_dim;
rate_y = min_y/y_dim;
rate_z = min_z/z_dim;
rate_x_y_z = min_x_y_z/x_y_z_dim;

%% Plot the graphs

figure(1);
clf;
plot(traj_t(:,1), traj_x(:,:,1), traj_t(:,no_of_elements), traj_x(:,:,no_of_elements));
xlabel('Time'),
title('Linear System 2');
legend('X','Y','Z');

figure(2);
axis equal;
grid;
plot3(traj_x(:,1,1),traj_x(:,2,1), traj_x(:,3,1), traj_x(:,1,no_of_elements),traj_x(:,2,no_of_elements), traj_x(:,3,no_of_elements), usafe(1),usafe(2),usafe(3),'-o');
xlabel('X'),
ylabel('Y'),
zlabel('Z'),

figure(3);
axis equal;
grid;
plot3(traj_x(:,1,2), traj_x(:,2,2), traj_x(:,3,2), traj_x(:,1,3), traj_x(:,2,3), traj_x(:,3,3), traj_x(:,1,5), traj_x(:,2,5), traj_x(:,3,5), usafe(1), usafe(2), usafe(3),'-o');
xlabel('X'),
ylabel('Y'),
zlabel('Z'),
legend('Z','Y','X');

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


