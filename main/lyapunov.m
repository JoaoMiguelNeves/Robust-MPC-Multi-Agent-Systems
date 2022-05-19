function [x, error_pos, energy_spent, distance] = lyapunov(n, tmax, m, formation, obstacleCircles)
% Solves the optimization problem using the CLF method
	
	Ai = kron([0 1;0 0], eye(2));	% double integrator ith node dynamics
	Bi = kron([0;1],eye(2));		% actuation affects both current velocity *and* current position
	
	G = c2d(ss(Ai,Bi,eye(m),0),0.5);

	A = kron(eye(n),G.A);			% MAS dynamics
	B = kron(eye(n),G.B);
	
	Ac = diag([1 1 0 0]) * G.A;			% MAS collision dynamics
	Bc = kron([1 1; 0 0], eye(m/2))* G.B;
	
	nx = n*m;           % number of states of MAS
	nu = n*size(Bi,2);  % number of actuatiors of the MAS

	x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
	u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'
	
	% actuation variable
	usdp = sdpvar(nu,1); 
	% relaxation variable 
	omega = sdpvar(1);
	
	% cost multiplier
	k = 1;
	% relaxation multiplier
	relax = 0.1;
	
	% measurements to assess simulation
	error_pos = zeros(1, tmax);
	energy_spent = zeros(1, tmax);
	distance = zeros(1, tmax);
	
	% Skew value
	err = 0.05;
	Ei = kron([0;1], eye(2));
	E = kron(eye(n), Ei);
	
	collision_x = [];
	r = [];
	% matrix to define the obstacle
	if ~isempty(obstacleCircles)
		for c = 1:size(obstacleCircles,1)
			circle = obstacleCircles(c,:);
			collision_x = [collision_x, repmat([circle(1) circle(2) 0 0].', n, 1)];
			r = [r, circle(3) + 1];
		end
	end
	
	% define solver settings
	S = sdpsettings('solver', 'mosek', 'cachesolvers', 1);
	S = sdpsettings(S, 'verbose', 0);

	V = kron(eye(n), diag([1 1 0 0]));
	
	for iter = 1:tmax		
		% Definition of the cost function for the MPC
		x_ref = x(:,iter) - formation(:,iter);		

		% Constraints for the optimization (CLF)
		constraints =	[(x_ref.' * A * x_ref) + (x_ref.' * B * usdp) <= omega,...
						-10 <= usdp, usdp <= 10];
					
		% Constraints for collision avoidance (CBF)
		if ~isempty(obstacleCircles)
			for c = 1:size(obstacleCircles,1)
				x_col = x(:,iter) - collision_x(:,c);
				for i = 1:n
					x_pos = x_col((i-1)*m +1:(i-1)*m +4);
					constraints = [constraints, (x_pos.' * Ac * x_pos) + (x_pos.' * Bc * usdp((i-1)*(m/2) + 1: (i-1)*(m/2) + 2)) - r(c)^2 >= 0];
				end
			end
		end
		
		% minimize the actuation in order to achieve the objective
		f = k * (norm(usdp)^2) + relax * omega^2;

		optimize(constraints, f, S);

		u(:,iter) = value(usdp);
		
		% update MAS state using the MPC controller
		x(:,iter+1) = A * x(:,iter) + B * u(:,iter); 

		% add an error to the agent's velocity
		if err ~= 0
			random_error = -err + (2*err) * rand(n*2,1);
			x(:,iter+1) = x(:,iter+1) + E * random_error;
		end
		
		if (iter == 1) 
			error_pos(iter) = norm(x_ref.' * kron(eye(n),diag([1 1 0 0])));
			energy_spent(iter) = norm(u(:,iter));
			distance(iter) = norm(V *(x(:,iter+1) - x(:,iter)));
		else 
			% energy spent and distance are cumulatively saved
			x_err = x(:,iter) - formation(:,iter-1);
			error_pos(iter) = norm(x_err.' * kron(eye(n),diag([1 1 0 0])));
			energy_spent(iter) = energy_spent(iter - 1) + norm(u(:,iter));
			distance(iter) = distance(iter - 1) + norm(V * (x(:,iter+1) - x(:,iter)));
		end
	end
end
	