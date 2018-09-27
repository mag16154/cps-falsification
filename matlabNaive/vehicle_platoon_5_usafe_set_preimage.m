%% 15 dimensional linear system - vehicle_platoon_5 
%% vehicle platoon

%% Apply the superposition principle to represent the distance vector 
%% to the unsafe set as a linear combination of the basis vectors.

clear all;
clc;

format shortg;

%% Dimensionality of the system
no_of_dimensions = 15;

%% Initial conditions/variables
init_variables = ones(no_of_dimensions, 2);
init_variables(1,:) = [0.9 1.1];
init_variables(2,:) = [0.9 1.1];
init_variables(3,:) = [0.9 1.1];
init_variables(4,:) = [0.9 1.1];
init_variables(5,:) = [0.9 1.1];
init_variables(6,:) = [0.9 1.1];
init_variables(7,:) = [0.9 1.1];
init_variables(8,:) = [0.9 1.1];
init_variables(9,:) = [0.9 1.1];
init_variables(10,:) = [0.9 1.1];
init_variables(11,:) = [0.9 1.1];
init_variables(12,:) = [0.9 1.1];
init_variables(13,:) = [0.9 1.1];
init_variables(14,:) = [0.9 1.1];
init_variables(15,:) = [0.9 1.1];

%init_elements = transpose(combvec(init_variables(1,:), init_variables(2,:), init_variables(3,:), init_variables(4,:), init_variables(5,:), init_variables(6,:), init_variables(7,:), init_variables(8,:), init_variables(9,:), init_variables(10,:), init_variables(11,:), init_variables(12,:), init_variables(13,:), init_variables(14,:), init_variables(15,:)));

init_elements=zeros(no_of_dimensions+2, no_of_dimensions);

%% First(center) and last initial states
for idn = 1:no_of_dimensions;
	init_elements(1,idn) = init_variables(idn,1);
	init_elements(no_of_dimensions+2,idn) = init_variables(idn,2);
end

%% Rest of the states
for idn = 1:no_of_dimensions;
	for idx = 1:no_of_dimensions;
		if (idx == idn);
			init_elements(idn+1, idx) = init_variables(idx,2);
		else
			init_elements(idn+1, idx) = init_variables(idx,1);
		end
	end
end
[A_init b_init] = vert2lcon(init_elements);

%% Unsafe conditions
usafe_variables = ones(no_of_dimensions,2);
usafe_variables(1,:) = [-0.5, 0.91]; 
usafe_variables(2,:) = [-0.5, 0.91]; 
usafe_variables(3,:) = [-0.5, 0.91]; 
usafe_variables(4,:) = [-0.5, 0.91]; 
usafe_variables(5,:) = [-0.5, 0.94]; 
usafe_variables(6,:) = [-0.5, 0.98]; 
usafe_variables(7,:) = [-0.5, 0.99]; 
usafe_variables(8,:) = [-0.5, 0.91]; 
usafe_variables(9,:) = [-0.5, 0.91]; 
usafe_variables(10,:) = [-0.5, 0.91]; 
usafe_variables(11,:) = [-0.5, 0.93]; 
usafe_variables(12,:) = [-0.5, 0.91]; 
usafe_variables(13,:) = [-0.5, 0.92]; 
usafe_variables(14,:) = [-0.5, 0.91]; 
usafe_variables(15,:) = [-0.5, 0.91]; 

k = 1;

%% Compute the pre image of the unsafe set
usafe_preimage = compute_usafe_preimage(no_of_dimensions, init_variables, usafe_variables, k);

%% Number of vertices/corners of the hyperplane representing the unsafe set
no_of_usafe_elements = size(usafe_preimage,3);

%% vert2con accepts vertices represented in 2 dimensional form
usafe_preimage_2D = zeros(no_of_usafe_elements,no_of_dimensions);

for i=1:no_of_usafe_elements;
	usafe_preimage_2D(i,:) = usafe_preimage(1,:,i);
end
[A_usafe b_usafe] = vert2lcon(usafe_preimage_2D);	
%Intersection_vertices = lcon2vert([A_init;A_usafe], [b_init;b_usafe]);
%tt Compute the intersection
%I=intersectionHull('vert',init_elements,'vert',usafe_preimage_2D); 

% ============================================================================================
% compute_usafe_preimage with respect to the simulation at k_time_stamp
% ============================================================================================
function usafe_preimage_at_k = compute_usafe_preimage(no_of_dimensions, init_variables, usafe_variables, k_time_stamp)

%% No of initial states to be simulated 
%% Extra two simulations are for the center and
%% the state reachable after having moved along each dim
no_of_init_elements= no_of_dimensions+2;
no_of_usafe_elements = no_of_dimensions+2;

%% Populating the array of unsafe states
usafe_state_array=zeros(no_of_usafe_elements, no_of_dimensions);

%% First(center) and last usafe states
for idn = 1:no_of_dimensions;
	usafe_state_array(1,idn) = usafe_variables(idn,1);
	usafe_state_array(no_of_usafe_elements,idn) = usafe_variables(idn,2);
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
%usafe_elements = combvec(usafe_variables(1,:), usafe_variables(2,:), usafe_variables(3,:), usafe_variables(4,:), usafe_variables(5,:), usafe_variables(6,:), usafe_variables(7,:), usafe_variables(8,:), usafe_variables(9,:), usafe_variables(10,:), usafe_variables(11,:), usafe_variables(12,:), usafe_variables(13,:), usafe_variables(14,:), usafe_variables(15,:));

%no_of_usafe_elements = size(usafe_elements,2);

%usafe_state_array = transpose(usafe_elements);

%% Populating the array of initial states
%% array_index -> state
%% 1 -> (x1,y1,z1) -- init/center state
%% 2 -> (x2,y1,z1) -- x changed
%% 3 -> (x1,y2,z1) -- y changed
%% 4 -> (x1,y1,z2) -- z changed
%% 5 -> (x2,y2,z2) -- all changed
init_state_array=zeros(no_of_init_elements, no_of_dimensions);

%% First(center) and last initial states
for idn = 1:no_of_dimensions;
	init_state_array(1,idn) = init_variables(idn,1);
	init_state_array(no_of_init_elements,idn) = init_variables(idn,2);
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
tspan = 0:0.05:20;

%% Integration/Simulation
for idx = 1:no_of_init_elements
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
dist_center_usafe = zeros(size(traj_t,1), no_of_dimensions, no_of_usafe_elements);

%% Euclidean distance
%dist_center_usafe_euclid = zeros(size(traj_t,1), no_of_usafe_elements);
for idn = 1:no_of_usafe_elements;
	for idt = 1:size(traj_t,1);
		pair(1,:) = traj_x(idt,:,1);
		pair(2,:) = usafe_state_array(idn,:);
		dist_center_usafe(idt,:,idn) = pair(2,:) - pair(1,:);
%		dist_center_usafe_euclid(idt,idn) = pdist(pair,'euclidean');
	end
end

%% Compute the base vectors (co-ordinate form) along each dimension at each time step
base_vectors = ones(size(traj_t,1), no_of_dimensions, no_of_dimensions);
for idt = 1:size(traj_t,1);
	for idn = 1:no_of_dimensions;
		pair(1,:) = traj_x(idt,:,1);
		pair(2,:) = traj_x(idt,:,idn+1);
		base_vectors(idt,:,idn) = pair(2,:) - pair(1,:);
		base_vectors_euclid(idt,idn) = pdist(pair, 'euclidean');
	end
end

max_base_vectors = max(base_vectors_euclid);
min_base_vectors = min(base_vectors_euclid);

gradient_base_vectors = min_base_vectors./max_base_vectors;

for idt = 1:size(traj_t,1);
	for idn = 1:no_of_dimensions;
		if (base_vectors_euclid(idt,idn) < gradient_base_vectors(idn));
			base_vectors_ignore(idt) = 1;
		end
	end
end

beta_time_counts = 0;
for idt = 1:size(traj_t,1);
	if (base_vectors_ignore(idt) ~= 1);
		beta_time_counts = beta_time_counts + 1;
	end
end

beta_end_time = 0.05*(beta_time_counts-1);
beta_time_span = 0:0.05:beta_end_time;

%% Compute the beta's (no_of_dimensions) for each usafe point (no_of_elements)
%% at every time step
beta = zeros(beta_time_counts, no_of_dimensions, no_of_usafe_elements);
for idt = 1:size(traj_t,1);
	if (base_vectors_ignore(idt) == 1);
		continue;
	end
	%% Helps getting away the syntax error while computing inverse
	for idn=1:no_of_dimensions;
		base_vector_at_idt(:,idn) = base_vectors(idt,:,idn);
	end

	%% Inverse of the base vector at time step, idt
	inv_base(:,:) = inv(base_vector_at_idt(:,:));
	
	%% beta = inv_base * distance
	for idn=1:no_of_usafe_elements;
		beta(idt,:,idn) = (inv_base(:,:)*(transpose(dist_center_usafe(idt,:,idn))));
	end
end

usafe_preimage = ones(beta_time_counts, no_of_dimensions, no_of_usafe_elements);
for idn = 1:no_of_dimensions;
	init_base_vectors(:,idn) = base_vectors(1,:,idn);
end

for idt = 1:beta_time_counts;
	for idn = 1:no_of_usafe_elements;
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

%% variables

e1=v(1);
e1prime=v(2);
a1=v(3);
e2=v(4);
e2prime=v(5);
a2=v(6);
e3=v(7);
e3prime=v(8);
a3=v(9);
e4=v(10);
e4prime=v(11);
a4=v(12);
e5=v(13);
e5prime=v(14);
a5=v(15);

%% equations

dv = [
  1*e1prime;
  -a1;
  1.7152555329*e1 + 3.9705119979*e1prime - 4.3600526739*a1 - 0.9999330812*e2 - 1.5731541104*e2prime + 0.2669165553*a2 - 0.2215507198*e3 - 0.4303855023*e3prime + 0.0669078193*a3 - 0.0881500219*e4 - 0.1881468451*e4prime + 0.0322187056*a4 - 0.0343095071*e5 - 0.0767587194*e5prime + 0.0226660281*a5;
  e2prime;
  a1 - a2;
  0.7153224517*e1 + 2.3973578876*e1prime + 0.2669165553*a1 + 1.4937048131*e2 + 3.5401264957*e2prime - 4.2931448546*a2 - 1.0880831031*e3 - 1.7613009555*e3prime + 0.2991352608*a3 - 0.2558602268*e4 - 0.5071442217*e4prime + 0.0895738474*a4 - 0.0881500219*e5 - 0.1881468451*e5prime + 0.0548847337*a5;
  e3prime;
  a2 - a3;
  0.493771732*e1 + 1.9669723853*e1prime + 0.0669078193*a1 + 0.6271724298*e2 + 2.2092110425*e2prime + 0.2991352608*a2 + 1.4593953061*e3 + 3.4633677762*e3prime - 4.2704788265*a3 - 1.0880831031*e4 - 1.7613009555*e4prime + 0.3218012889*a4 - 0.2215507198*e5 - 0.4303855023*e5prime + 0.121792553*a5;
  e4prime + 0;
  a3 - a4;
  0.40562171*e1 + 1.7788255402*e1prime + 0.0322187056*a1 + 0.4594622249*e2 + 1.8902136659*e2prime + 0.0895738474*a2 + 0.6271724298*e3 + 2.2092110425*e3prime + 0.3218012889*a3 + 1.4937048131*e4 + 3.5401264957*e4prime - 4.2382601209*a4 - 0.9999330812*e5 - 1.5731541104*e5prime + 0.3887091083*a5;
  e5prime;
  a4 - a5;
  0.371312203*e1 + 1.7020668208*e1prime + 0.0226660281*a1 + 0.40562171*e2 + 1.7788255402*e2prime + 0.0548847337*a2 + 0.493771732*e3 + 1.9669723853*e3prime + 0.121792553*a3 + 0.7153224517*e4 + 2.3973578876*e4prime + 0.3887091083*a4 + 1.7152555329*e5 + 3.9705119979*e5prime - 3.9713435656*a5;
] ;
end
