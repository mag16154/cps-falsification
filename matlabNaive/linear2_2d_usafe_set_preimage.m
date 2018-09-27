%% 2 dimensional linear system with dynamics
%% dx = -x, dy = x-y

%% Apply the superposition principle to represent the distance vector 
%% to the unsafe set as a linear combination of the basis vectors.

clear all;
clc;

format shortg;

%% Dimensionality of the system
no_of_dimensions = 2;
no_of_elements = 4;

%% Initial conditions/variables
%% Values later will be designated as 
%% x1, x2, y1, y2.

init_variables = ones(no_of_dimensions, 2);
init_variables(1,:) = [4 9];
init_variables(2,:) = [1 6];

%% Co-ordinates of the unsafe states
usafe_variables = ones(no_of_dimensions,2);
usafe_variables(1,:) = [5 5.5];
usafe_variables(2,:) = [3 3.5];

k = 4;

usafe_preimage = compute_usafe_preimage(no_of_dimensions, init_variables, usafe_variables,k);

%% vert2con accepts 2 dim array of vertices
usafe_preimage_2D = zeros(no_of_elements,no_of_dimensions);

for i=1:no_of_elements;
	usafe_preimage_2D(i,:) = usafe_preimage(1,:,i);
end
[A b] = vert2con(usafe_preimage_2D);	
%K1 = convhulln(usafe_preimage_cvhull);
% ============================================================================================
% compute_usafe_preimage
% ============================================================================================
function usafe_preimage_at_k = compute_usafe_preimage(no_of_dimensions, init_variables, usafe_variables, k_time_stamp)

%% No of initial states to be simulated 
%% or the number of usafe states

%% Extra two simulations are for the center and
%% the state reachable after having moved along each dim
no_of_elements= no_of_dimensions+2;

%% Populating the array of unsafe states
usafe_state_array=zeros(no_of_elements, no_of_dimensions);

%% First(center) and last usafe states
for idn = 1:no_of_dimensions;
	usafe_state_array(1,idn) = usafe_variables(idn,1);
	usafe_state_array(no_of_elements,idn) = usafe_variables(idn,2);
end

%% Rest of the usafe states
for idn = 1:no_of_dimensions;
	for idx = 1:no_of_dimensions;
		if (idx == idn);
			usafe_state_array(idn+1, idx) = usafe_variables(idx,2);
		else
			usafe_state_array(idn+1, idx) = usafe_variables(idx,1);
		end
	end
end

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
		pair(2,:) = usafe_state_array(idn,:);
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
	
	%% Helps getting away with the syntax error while computing inverse
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


usafe_preimage = ones(size(traj_t,1), no_of_dimensions, no_of_elements);
for idn = 1:no_of_dimensions;
	init_base_vectors(:,idn) = base_vectors(1,:,idn);
end
for idt = 1:size(traj_t,1);
	for idn = 1:no_of_elements;
		beta_at_idn(:) = beta(idt,:,idn);
		displacement(:) = beta_at_idn*init_base_vectors(:,:);
		central_traj_state = traj_x(idt,:,1);
		usafe_preimage(idt,:,idn) = central_traj_state+displacement;
	end
end

usafe_preimage_at_k = usafe_preimage(k_time_stamp,:,:);
end

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
