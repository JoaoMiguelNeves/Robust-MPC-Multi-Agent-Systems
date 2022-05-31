function [x, error_pos, energy_spent, distance] = lqr(n, tmax, m, formation)
% Solves the optimization problem using the lqr method

	Ai = kron([1 1;0 1], eye(2)); % double integrator ith node dynamics
	Bi = kron([0;1],eye(2));   

	A = kron(eye(n),Ai); % MAS dynamics
	B = kron(eye(n),Bi);
    
	nx = n*m;           % number of states of MAS
	nu = n*size(Bi,2);  % number of actuatiors of the MAS

	x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
	u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'
	x(:,1) = rand(nx,1);
	
	% measurements to assess simulation
	error_pos = zeros(1, tmax);
	energy_spent = zeros(1, tmax);
	distance = zeros(1, tmax);
	
	V = kron(eye(n), diag([1 1 0 0]));

	for iter = 1:tmax
		% Discrete-time LQR (infinite horizon without constraints) controller
		% error in the formation
		e = x(:,iter) - formation(:,iter);
		[K,S,E] = dlqr(A,B,1000*eye(nx),1*eye(nu));

		% select actuation
		u(:,iter) = -K * e;
		
		% update MAS state 
		x(:,iter+1) = A * x(:,iter) + B * u(:,iter);
		
		if (iter == 1) 
			error_pos(iter) = norm(e.' * kron(eye(n),diag([1 1 0 0])));
			energy_spent(iter) = norm(u(:,iter));
			distance(iter) = norm(V *(x(:,iter+1) - x(:,iter)));
		else 
			% energy spent and distance are cumulatively saved
			x_err = x(:,iter) - formation(:,iter);
			error_pos(iter) = norm(x_err.' * kron(eye(n),diag([1 1 0 0])));
			energy_spent(iter) = energy_spent(iter - 1) + norm(u(:,iter));
			distance(iter) = distance(iter - 1) + norm(V * (x(:,iter+1) - x(:,iter)));
		end
	end
end

