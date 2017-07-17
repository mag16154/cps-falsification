%% 2 dimensional linear system with dynamics
%% dx = -x, dy = x-y

%% Apply the superposition principle to represent the distance vector 
%% to the unsafe set as a linear combination of the basis vectors.

clear all;
clc;

format shortg;

%% Dimensionality of the system
no_of_dimensions = 2;

%% Initial conditions/variables
%% Values later will be designated as 
%% x1, x2, y1, y2.

init_variables = ones(no_of_dimensions, 2);
init_variables(1,:) = [4 9];
init_variables(2,:) = [1 6];

%% Co-ordinates of the unsafe states
usafe = [5 3; 5 3.5; 5.5 3; 5.5 3.5];

%% Base vector at time step 1  
%% To be used later in the code to find displacement
for idn=1:no_of_dimensions;
	init_base_vector(idn) = init_variables(idn,2) - init_variables(idn,1);
end

%% No of initial states to be simulated 
%% or the number of usafe states

%% Extra two simulations are for the center and
%% the state reachable after having moved along each dim
no_of_elements= no_of_dimensions+2;

%% Populating the array of initial states
%% array_index -> state
%% 1 -> (x1,y1) -- init/center state
%% 2 -> (x2,y1) -- x changed
%% 3 -> (x1,y2) -- y changed
%% 4 -> (x2,y2) -- both changed
init_state_array=zeros(no_of_elements, no_of_dimensions);

%% First(center) and last initial states
for idn = 1:no_of_dimensions;
	init_state_array(1,idn) = init_variables(idn,1);
	init_state_array(no_of_elements,idn) = init_variables(idn,2);
end

%% Rest of the states
for idn = 1:no_of_dimensions;
	for idx = 1:no_of_dimensions;
		if (idx == idn);
			init_state_array(idn+1, idx) = init_variables(idx,2);
		else
			init_state_array(idn+1, idx) = init_variables(idx,1);
		end
	end
end

%% Simulation Time
tspan = 0:0.02:5;

%% Integration/Simulation
for idx = 1:no_of_elements
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	
	%% Simulations' time stamps
	%% Doesn't vary if fixed time steps
	traj_t(:,idx) = t;
	
	%% Simulations
	%% First dimesion is time 
	%% 2nd is for variables 
	traj_x(:,:,idx) = x; 
end

%% Compute the distance from the central trajectory to all the usafe points

%% The distance in co-ordinate form
dist_center_usafe = zeros(size(traj_t,1), no_of_dimensions, no_of_elements);

%% Euclidean distance
dist_center_usafe_euclid = zeros(size(traj_t,1), no_of_elements);
for idn = 1:no_of_elements;
	for idt = 1:size(traj_t,1);
		pair(1,:) = traj_x(idt,:,1);
		pair(2,:) = usafe(idn,:);
		dist_center_usafe(idt,:,idn) = pair(2,:) - pair(1,:);
		dist_center_usafe_euclid(idt,idn) = pdist(pair,'euclidean');
	end
end

%% Compute min distance to each unsafe point, and also corresponding time step
%% Just to check if the distance to each usafe point has indeed reduced from the 
%% previous run/beta's
for idn = 1:no_of_elements;
	min_dist(idn) = dist_center_usafe_euclid(1,idn);
	min_dist_time_step(idn) = 1;
	for idt = 2:size(traj_t,1);
		if(min_dist(idn) > dist_center_usafe_euclid(idt,idn));
			min_dist(idn) = dist_center_usafe_euclid(idt,idn);
			min_dist_time_step(idn) = idt;
		end 
	end
end

%% Compute the base vectors (co-ordinate form) along each dimension at each time step
base_vectors = ones(size(traj_t,1), no_of_dimensions, no_of_dimensions);
for idt = 1:size(traj_t,1);
	for idn = 1:no_of_dimensions;
		pair(1,:) = traj_x(idt,:,1);
		pair(2,:) = traj_x(idt,:,idn+1);
		base_vectors(idt,:,idn) = pair(2,:) - pair(1,:);
	end
end

%% Compute the beta's (no_of_dimensions) for each usafe point (no_of_elements)
%% at every time step
beta = ones(size(traj_t,1), no_of_dimensions, no_of_elements);
for idt = 1:size(traj_t,1);
	
	%% Helps getting away the syntax error while computing inverse
	for idn=1:no_of_dimensions;
		base_vector_at_idt(:,idn) = base_vectors(idt,:,idn);
	end

	%% Inverse of the base vector at time step, idt
	inv_base(:,:) = inv(base_vector_at_idt(:,:));
	
	%% beta = inv_base * distance
	for idn=1:no_of_elements;
		beta(idt,:,idn) = (inv_base(:,:)*transpose(dist_center_usafe(idt,:,idn)));
	end
end

%% Choose max-beta (out of no_of_dimensions) at each time step for each unsafe point
max_beta_array = ones(size(traj_t,1), no_of_dimensions);
for idt = 1:size(traj_t,1);
	for idn = 1:no_of_elements;
		max_beta_array(idt,idn) = abs(beta(idt,1,idn));
		for idx = 2:no_of_dimensions;
			if (max_beta_array(idt,idn) < abs(beta(idt,idx,idn)));
				max_beta_array(idt,idn) = abs(beta(idt,idx,idn));
			end
		end
	end
end

%% Find minimum beta and the time step for each unsafe point
for idn = 1:no_of_elements;
	min_beta_vector(idn) = max_beta_array(1,idn);
	min_beta_time_step_vector(idn) = 1;
	for idt = 2:size(traj_t,1);
		if (min_beta_vector(idn) > max_beta_array(idt,idn))
			min_beta_vector(idn) = max_beta_array(idt,idn);
			min_beta_time_step_vector(idn) = idt;
		end
	end
end

%% Take the minimum beta, it's time step and the corresponding usafe point (direction) 
min_beta = min_beta_vector(1);
min_beta_time_step = min_beta_time_step_vector(1);
min_beta_direction = 1;
for idn= 2:no_of_elements;
	if (min_beta > min_beta_vector(idn));
		min_beta = min_beta_vector(idn);
		min_beta_time_step = min_beta_time_step_vector(idn);
		min_beta_direction = idn;
	end
end

%% Compute the next init state for next run
next_step_beta = beta(min_beta_time_step, :, min_beta_direction);
displacement = init_base_vector.*next_step_beta;
next_init_state = init_state_array(1,:) + displacement;

%% Plot max beta at each time step
figure(1);
plot(traj_t(:,1), max_beta_array(:,:));
xlabel('Time'),
ylabel('Beta'),

% ============================================================================================
% dvdt
% ============================================================================================

function dv = dxdt(t,v)

%% parameter set 1

k1=1;
k2=1;

%% variables

x=v(1);
y=v(2);

%% equations

dv = [
    -k1*x;    	% dx/dt
    k1*x-k2*y;  % dy/dt
] ;
end
