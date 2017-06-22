
%% 2 dimensional linear system with dynamics
%% dx = -x, dy = x-y

%% Consider just the distance from the unsafe state
%% Compute the number of times a simulation is closer 
%% to the usafe state along each dimension.

clear all;
clc;

%% Initial conditions
%% Values later will be designated as 
%% x1, x2, x3, y1, y2, y3.

X = 8.5:0.5:9.5;
Y = 5.5:0.5:6.5;

%% Co-ordinates of an unsafe state
usafe = [2.4;4.8];

%% Total number of initial states to be simulated
no_of_elements=numel(X)*numel(Y);

%% Populating the array of initial states
%% array_index -> state
%% 1 -> (x1,y1) -- init/center state
%% 2 -> (x1,y2)
%% 3 -> (x1,y3) -- x unchanged and y changed
%% 4 -> (x2,y1)
%% 5 -> (x2,y2)
%% 6 -> (x2,y3)
%% 7 -> (x3,y1) -- x changed and y unchanged
%% 8 -> (x3,y2)
%% 9 -> (x3,y3)

init_state_array=ones(no_of_elements,2);
for myx=1:numel(X)
	for myy=1:numel(Y)
		x_idx=numel(Y)*(myx-1)+myy;
		init_state_array(x_idx,1) = X(myx);
		init_state_array(x_idx,2) = Y(myy);
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

%% Compute the distance for the trajectory@7 where x is changed 
%% and y is unchanged.
%% Also compute the base vectors, just in case.

idy = [1 7];
for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,idy(i+1));
		pair(2,:) = traj_x(idx,:,idy(1));
		base_vectors_x(idx,i) = pdist(pair,'euclidean');
		pair(2,:) = usafe(:);
		dist_x_usafe(idx,i) = pdist(pair,'euclidean');
	end
end

%% Compute the distance for the trajectory@3 where x is unchanged 
%% and y is changed.
%% Also compute the base vectors, just in case.

idy = [1 3];
for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,idy(i+1));
		pair(2,:) = traj_x(idx,:,idy(1));
		base_vectors_y(idx,i) = pdist(pair,'euclidean');
		pair(2,:) = usafe(:);
		dist_y_usafe(idx,i) = pdist(pair,'euclidean');
	end
end	

%% Compute the distance for the trajectory@9 where both x and y 
%% are changed.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		pair(1,:) = traj_x(idx,:,no_of_elements);
		pair(2,:) = usafe(:);
		dist_x_y_usafe(idx,i) = pdist(pair,'euclidean');
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

%% Keep a track of the number of times the trajectory by changing x and y 
%% is perform than that the one from the central state.

for idx = 1:size(traj_t,1);
	for i = 1:1;
		if (dist_center_usafe(idx,i) < dist_x_y_usafe(idx,i))
			pick_x_y(idx,i) = 0;
		else
			pick_x_y(idx,i) = 1;
		end
	end
end

%% Count the numbers along each direction

y_dim = 0;
x_dim = 0;
x_y_dim = 0;
for idx = i:size(traj_t,1);
	if (pick_x(idx,1) == 1)
		x_dim = x_dim+1;
	end
	if (pick_y(idx,1) == 1)
		y_dim = y_dim+1;	
	end
	if (pick_x_y(idx,i) == 1)
		x_y_dim = x_y_dim+1;
	end
end

%% Find the minimum along each direction

min_x = dist_x_usafe(idx,1);
min_y = dist_y_usafe(idx,1);
min_x_y = dist_x_y_usafe(idx,1);
min_init = dist_center_usafe(idx,1);
for idx = 2:size(traj_t,1);
	if (min_x > dist_x_usafe(idx,1))
		min_x = dist_x_usafe(idx,1);
	end
	if (min_y > dist_y_usafe(idx,1))
		min_y = dist_y_usafe(idx,1);
	end
	if (min_x_y > dist_x_y_usafe(idx,1))
		min_x_y = dist_x_y_usafe(idx,1);
	end
	if (min_init > dist_center_usafe(idx,1))
		min_init = dist_center_usafe(idx,1);
	end
end

%% Compute the rate

rate_x = min_x/x_dim;
rate_y = min_y/y_dim;
rate_x_y = min_x_y/x_y_dim;

%% Plot the graphs

figure(1);
clf;
plot(traj_t(:,1), traj_x(:,:,1), traj_t(:,no_of_elements), traj_x(:,:,no_of_elements));
xlabel('Time'),
title('Linear System 2');
legend('X','Y');

figure(2);
plot(traj_x(:,1,1),traj_x(:,2,1), traj_x(:,1,no_of_elements),traj_x(:,2,no_of_elements), usafe(1),usafe(2),'-o');
xlabel('X'),
ylabel('Y'),

figure(3);
plot(traj_x(:,1,3), traj_x(:,2,3), traj_x(:,1,7), traj_x(:,2,7), usafe(1), usafe(2), '-o');
xlabel('X'),
ylabel('Y'),
legend('Y','X');

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


