%% 30 dimensional linear system - vehicle_platoon_5 
%% vehicle platoon

%% Apply the superposition principle to represent the distance vector 
%% to the unsafe set as a linear combination of the basis vectors.

clear all;
clc;

format shortg;

%% Dimensionality of the system
no_of_dimensions = 30;

%% No of initial states to be simulated 
%% or the number of usafe states

%% Extra two simulations are for the center and
%% the state reachable after having moved along each dim
no_of_elements= no_of_dimensions+2;

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
init_variables(16,:) = [0.9 1.1];
init_variables(17,:) = [0.9 1.1];
init_variables(18,:) = [0.9 1.1];
init_variables(19,:) = [0.9 1.1];
init_variables(20,:) = [0.9 1.1];
init_variables(21,:) = [0.9 1.1];
init_variables(22,:) = [0.9 1.1];
init_variables(23,:) = [0.9 1.1];
init_variables(24,:) = [0.9 1.1];
init_variables(25,:) = [0.9 1.1];
init_variables(26,:) = [0.9 1.1];
init_variables(27,:) = [0.9 1.1];
init_variables(28,:) = [0.9 1.1];
init_variables(29,:) = [0.9 1.1];
init_variables(30,:) = [0.9 1.1];

%% Unsafe conditions
usafe_variables = ones(no_of_dimensions,2);
usafe_variables(1,:) = [-0.5, -0.2]; 
usafe_variables(2,:) = [-0.5, -0.2]; 
usafe_variables(3,:) = [-0.5, -0.2]; 
usafe_variables(4,:) = [-0.5, -0.2]; 
usafe_variables(5,:) = [-0.5, -0.2]; 
usafe_variables(6,:) = [-0.5, -0.2]; 
usafe_variables(7,:) = [-0.5, -0.2]; 
usafe_variables(8,:) = [-0.5, -0.2]; 
usafe_variables(9,:) = [-0.5, -0.2]; 
usafe_variables(10,:) = [-0.5, -0.2]; 
usafe_variables(11,:) = [-0.5, -0.2]; 
usafe_variables(12,:) = [-0.5, -0.2]; 
usafe_variables(13,:) = [-0.5, -0.2]; 
usafe_variables(14,:) = [-0.5, -0.2]; 
usafe_variables(15,:) = [-0.5, -0.2]; 
usafe_variables(16,:) = [-0.5, -0.2]; 
usafe_variables(17,:) = [-0.5, -0.2]; 
usafe_variables(18,:) = [-0.5, -0.2]; 
usafe_variables(19,:) = [-0.5, -0.2]; 
usafe_variables(20,:) = [-0.5, -0.2]; 
usafe_variables(21,:) = [-0.5, -0.2]; 
usafe_variables(22,:) = [-0.5, -0.2]; 
usafe_variables(23,:) = [-0.5, -0.2]; 
usafe_variables(24,:) = [-0.5, -0.2]; 
usafe_variables(25,:) = [-0.5, -0.2]; 
usafe_variables(26,:) = [-0.5, -0.2]; 
usafe_variables(27,:) = [-0.5, -0.2]; 
usafe_variables(28,:) = [-0.5, -0.2]; 
usafe_variables(29,:) = [-0.5, -0.2]; 
usafe_variables(30,:) = [-0.5, -0.2]; 

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

%% Representation of the initial states
%% array_index -> state
%% 1 -> (x1,y1,z1) -- init/center state
%% 2 -> (x2,y1,z1) -- x changed
%% 3 -> (x1,y2,z1) -- y changed
%% 4 -> (x1,y1,z2) -- z changed
%% 5 -> (x2,y2,z2) -- all changed

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
tspan = 0:0.05:20;

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
beta = zeros(size(traj_t,1), no_of_dimensions, no_of_elements);
for idt = 1:size(traj_t,1);
	
	%% Helps getting away the syntax error while computing inverse
	for idn=1:no_of_dimensions;
		base_vector_at_idt(:,idn) = base_vectors(idt,:,idn);
	end

	%% Inverse of the base vector at time step, idt
	inv_base(:,:) = inv(base_vector_at_idt(:,:));
	
	%% beta = inv_base * distance
	for idn=1:no_of_elements;
		beta(idt,:,idn) = (inv_base(:,:)*(transpose(dist_center_usafe(idt,:,idn))));
	end
end

%% Find beta for min_distance_time_step to each unsafe point
%% Not used - Just for debugging.
beta_for_min_dist = zeros(no_of_dimensions, no_of_elements);
for idn = 1:no_of_elements;
	beta_for_min_dist(:, idn) = beta(min_dist_time_step(idn), :, idn);
end

%% Choose max-beta (out of no_of_dimensions) at each time step for each unsafe point
max_beta_array = zeros(size(traj_t,1), no_of_dimensions);
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

%% Base vector at time step 1  
%% To be used to find displacement
for idn=1:no_of_dimensions;
	init_base_vector(idn) = init_variables(idn,2) - init_variables(idn,1);
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

figure(2);
plot(traj_t(:,1), traj_x(:,:,1));

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
e6=v(16);
e6prime=v(17);
a6=v(18);
e7=v(29);
e7prime=v(20);
a7=v(21);
e8=v(22);
e8prime=v(23);
a8=v(24);
e9=v(25);
e9prime=v(26);
a9=v(27);
e10=v(28);
e10prime=v(29);
a10=v(30);

%% equations

dv = [
  e1prime;
  -a1;
  1.702423734*e1 + 3.929356551*e1prime - 4.3607983776*a1 - 1.01374489*e2 - 1.6167727749*e2prime + 0.2653009364*a2 - 0.2375199245*e3 - 0.4793543458*e3prime + 0.06412815*a3 - 0.1079326841*e4 - 0.2463610381*e4prime + 0.0276872161*a4 - 0.0605561959*e5 - 0.1501445039*e5prime + 0.0151944922*a5 - 0.0374830081*e6 - 0.0986391305*e6prime + 0.009628751*a6 - 0.0242136837*e7 - 0.0665592904*e7prime + 0.0067836913*a7 - 0.015601062*e8 - 0.0442510048*e8prime + 0.0052325207*a8 - 0.0093924696*e9 - 0.0272127915*e9prime + 0.0043984935*a9 - 0.0044278796*e10 - 0.0129879863*e10prime + 0.0040303349*a10;
  e2prime;
  a1 - a2;
  0.688678844*e1 + 2.3125837761*e1prime + 0.2653009364*a1 + 1.4649038095*e2 + 3.4500022052*e2prime - 4.2966702275*a2 - 1.1216775741*e3 - 1.863133813*e3prime + 0.2929881525*a3 - 0.2980761204*e4 - 0.6294988497*e4prime + 0.0793226422*a4 - 0.1454156921*e5 - 0.3450001686*e5prime + 0.0373159671*a5 - 0.0847698796*e6 - 0.2167037943*e6prime + 0.0219781835*a6 - 0.0530840701*e7 - 0.1428901352*e7prime + 0.0148612718*a7 - 0.0336061533*e8 - 0.0937720819*e8prime + 0.0111821848*a8 - 0.0200289416*e9 - 0.057238991*e9prime + 0.0092628557*a9 - 0.0093924696*e10 - 0.0272127915*e10prime + 0.0084288284*a10;
  e3prime;
  a2 - a3;
  0.4511589195*e1 + 1.8332294303*e1prime + 0.06412815*a1 + 0.5807461599*e2 + 2.066222738*e2prime + 0.2929881525*a2 + 1.4043476136*e3 + 3.2998577013*e3prime - 4.2814757354*a3 - 1.1591605822*e4 - 1.9617729435*e4prime + 0.3026169036*a4 - 0.3222898041*e5 - 0.6960581401*e5prime + 0.0861063336*a5 - 0.1610167541*e6 - 0.3892511733*e6prime + 0.0425484878*a6 - 0.0941623492*e7 - 0.2439165858*e7prime + 0.026376677*a7 - 0.0575119497*e8 - 0.1558781215*e8prime + 0.0188916067*a8 - 0.0336061533*e9 - 0.0937720819*e9prime + 0.0152125197*a9 - 0.015601062*e10 - 0.0442510048*e10prime + 0.0136613491*a10;
  e4prime;
  a3 - a4;
  0.3432262354*e1 + 1.5868683922*e1prime + 0.0276872161*a1 + 0.3906027236*e2 + 1.6830849264*e2prime + 0.0793226422*a2 + 0.5432631518*e3 + 1.9675836075*e3prime + 0.3026169036*a3 + 1.3801339299*e4 + 3.2332984109*e4prime - 4.274692044*a4 - 1.1747616442*e5 - 2.0060239482*e5prime + 0.3078494243*a5 - 0.3316822737*e6 - 0.7232709316*e6prime + 0.090504827*a6 - 0.1654446337*e7 - 0.4022391596*e7prime + 0.0465788228*a7 - 0.0941623492*e8 - 0.2439165858*e8prime + 0.0304070119*a8 - 0.0530840701*e9 - 0.1428901352*e9prime + 0.0232901001*a9 - 0.0242136837*e10 - 0.0665592904*e10prime + 0.0204450405*a10;
  e5prime;
  a4 - a5;
  0.2826700395*e1 + 1.4367238883*e1prime + 0.0151944922*a1 + 0.3057432273*e2 + 1.4882292617*e2prime + 0.0373159671*a2 + 0.3663890398*e3 + 1.616525636*e3prime + 0.0861063336*a3 + 0.5276620899*e4 + 1.9233326028*e4prime + 0.3078494243*a4 + 1.3707414603*e5 + 3.2060856194*e5prime - 4.2702935506*a5 - 1.1791895238*e6 - 2.0190119345*e6prime + 0.3118797592*a6 - 0.3316822737*e7 - 0.7232709316*e7prime + 0.094535162*a7 - 0.1610167541*e8 - 0.3892511733*e8prime + 0.0509773162*a8 - 0.0847698796*e9 - 0.2167037943*e9prime + 0.0356395326*a9 - 0.0374830081*e10 - 0.0986391305*e10prime + 0.0300737915*a10;
  e6prime;
  a5 - a6;
  0.2451870315*e1 + 1.3380847578*e1prime + 0.009628751*a1 + 0.2584563558*e2 + 1.3701645979*e2prime + 0.0219781835*a2 + 0.2901421653*e3 + 1.443978257*e3prime + 0.0425484878*a3 + 0.3569965702*e4 + 1.5893128445*e4prime + 0.090504827*a4 + 0.5232342102*e5 + 1.9103446165*e5prime + 0.3118797592*a5 + 1.3707414603*e6 + 3.2060856194*e6prime - 4.2662632156*a6 - 1.1747616442*e7 - 2.0060239482*e7prime + 0.3162782527*a7 - 0.3222898041*e8 - 0.6960581401*e8prime + 0.0997676827*a8 - 0.1454156921*e9 - 0.3450001686*e9prime + 0.0577610076*a9 - 0.0605561959*e10 - 0.1501445039*e10prime + 0.0452682837*a10;
  e7prime;
  a6 - a7;
  0.2209733477*e1 + 1.2715254674*e1prime + 0.0067836913*a1 + 0.2295859695*e2 + 1.2938337531*e2prime + 0.0148612718*a2 + 0.2490638862*e3 + 1.3429518064*e3prime + 0.026376677*a3 + 0.2857142857*e4 + 1.4309902707*e4prime + 0.0465788228*a4 + 0.3569965702*e5 + 1.5893128445*e5prime + 0.094535162*a5 + 0.5276620899*e6 + 1.9233326028*e6prime + 0.3162782527*a6 + 1.3801339299*e7 + 3.2332984109*e7prime - 4.2610306949*a7 - 1.1591605822*e8 - 1.9617729435*e8prime + 0.323061944*a8 - 0.2980761204*e9 - 0.6294988497*e9prime + 0.1093964337*a9 - 0.1079326841*e10 - 0.2463610381*e10prime + 0.0729554998*a10;
  e8prime;
  a7 - a8;
  0.2053722857*e1 + 1.2272744627*e1prime + 0.0052325207*a1 + 0.2115808781*e2 + 1.2443126759*e2prime + 0.0111821848*a2 + 0.2251580898*e3 + 1.2808457668*e3prime + 0.0188916067*a3 + 0.2490638862*e4 + 1.3429518064*e4prime + 0.0304070119*a4 + 0.2901421653*e5 + 1.443978257*e5prime + 0.0509773162*a5 + 0.3663890398*e6 + 1.616525636*e6prime + 0.0997676827*a6 + 0.5432631518*e7 + 1.9675836075*e7prime + 0.323061944*a7 + 1.4043476136*e8 + 3.2998577013*e8prime - 4.2514019439*a8 - 1.1216775741*e9 - 1.863133813*e9prime + 0.3382564362*a9 - 0.2375199245*e10 - 0.4793543458*e10prime + 0.1370836498*a10;
  e9prime;
  a8 - a9;
  0.1959798161*e1 + 1.2000616712*e1prime + 0.0043984935*a1 + 0.2009444061*e2 + 1.2142864764*e2prime + 0.0092628557*a2 + 0.2115808781*e3 + 1.2443126759*e3prime + 0.0152125197*a3 + 0.2295859695*e4 + 1.2938337531*e4prime + 0.0232901001*a4 + 0.2584563558*e5 + 1.3701645979*e5prime + 0.0356395326*a5 + 0.3057432273*e6 + 1.4882292617*e6prime + 0.0577610076*a6 + 0.3906027236*e7 + 1.6830849264*e7prime + 0.1093964337*a7 + 0.5807461599*e8 + 2.066222738*e8prime + 0.3382564362*a8 + 1.4649038095*e9 + 3.4500022052*e9prime - 4.2237147278*a9 - 1.01374489*e10 - 1.6167727749*e10prime + 0.4023845862*a10;
  e10prime;
  a9 - a10;
  0.1915519365*e1 + 1.1870736849*e1prime + 0.0040303349*a1 + 0.1959798161*e2 + 1.2000616712*e2prime + 0.0084288284*a2 + 0.2053722857*e3 + 1.2272744627*e3prime + 0.0136613491*a3 + 0.2209733477*e4 + 1.2715254674*e4prime + 0.0204450405*a4 + 0.2451870315*e5 + 1.3380847578*e5prime + 0.0300737915*a5 + 0.2826700395*e6 + 1.4367238883*e6prime + 0.0452682837*a6 + 0.3432262354*e7 + 1.5868683922*e7prime + 0.0729554998*a7 + 0.4511589195*e8 + 1.8332294303*e8prime + 0.1370836498*a8 + 0.688678844*e9 + 2.3125837761*e9prime + 0.4023845862*a9 + 1.702423734*e10 + 3.929356551*e10prime - 3.9584137913*a10;
] ;
end
