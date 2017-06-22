%% 3 dimensional linear system
%% Compute the robustness by changing values along one dimension

clear all;
clc;

%% Initial conditions:
X = 1.5:0.02:2.5;
Y = 0.5;
Z = 0.5:0.02:1.5;
usafe = [1.3;0.6;1.08];

no_of_elements=numel(X)*numel(Y)*numel(Z);

%% Array of combinations of X,Y and Z values
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

%%%% Time:

tspan = 0:0.01:10;

%%%% Integration/Simulation:

for idx = 1:no_of_elements
%	localy0 = [Y0(idx,1) Y0(idx,2)];
	init_state = init_state_array(idx,:);
	[t x] = ode45(@dxdt,tspan,init_state);
	traj_t(:,idx) = t;
	traj_x(:,:,idx) = x;
end

for idx = 1:no_of_elements;
	for idt = 1:size(traj_t,1);
		pair(1,:) = traj_x(idt,:,idx);
		pair(2,:) = usafe(:);
		dist_usafe(idx,idt) = pdist(pair,'euclidean');
	end
end

%% Compute min robustness for each trajectory
for idx = 1:no_of_elements
	min_dist = dist_usafe(idx,1);
	for idt = 2:size(traj_t,1)
		if (min_dist > dist_usafe(idx,idt))
			min_dist = dist_usafe(idx,idt);
		end
	end
	robust_vector(idx) = min_dist;
end

%% Find the min of all
min_robustness = robust_vector(1);
for idx = 2:no_of_elements
	if (min_robustness > robust_vector(idx))
		min_robustness = robust_vector(idx);
	end
end

%%%% Figure:

figure(1);
clf;
plot(traj_t(:,1), traj_x(:,:,1), traj_t(:,no_of_elements), traj_x(:,:,no_of_elements));
xlabel('Time'),
title('Linear System 2');
legend('X','Y','Z');

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


