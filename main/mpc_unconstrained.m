function [x, t_collision, error_pos, energy_spent, distance] = mpc_unconstrained(n, tmax, m, h, formation, obstacleCircles)
	% Solver to be used as a first run before adding collision constraints

	Ai = kron([0 1;0 0], eye(2)); % double integrator ith node dynamics
	Bi = kron([0;1],eye(2));
	
	G = c2d(ss(Ai,Bi,eye(m),0),0.5);
	
	A = kron(eye(n),G.A);			% MAS dynamics
	B = kron(eye(n),G.B);
    
	nx = n*m;           % number of states of MAS
	nu = n*size(Bi,2);  % number of actuatiors of the MAS

	x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
	u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'	
	
	xsdp = sdpvar(nx*(h+1),1);	% define the variable for the state
	usdp = sdpvar(nu*h,1);		% define the variable for the actuation
	
	Qx = 10*eye(nx*(h+1));
	
	Aconstraint = [zeros(nx,nx*(h+1));kron(eye(h),A),zeros(h*nx,nx)];
	Bconstraint = [zeros(nx,nu*h); kron(eye(h),B)];
	
	t_collision = 0;
	
	% measurements to assess simulation
	error_pos = zeros(1, tmax);
	energy_spent = zeros(1, tmax);
	distance = zeros(1, tmax);
	
	Li = kron([1 1; 0 1],eye(2));
	L = kron(eye(n), Li);
	
	V = kron(eye(n), diag([1 1 0 0]));
	
	S = sdpsettings('solver', 'mosek', 'cachesolvers', 1);
	S = sdpsettings(S, 'verbose', 0);

	% Run unobstructed MPC optimization if there isn't any previous one
	% saved, or if there are no obstacles
	for iter = 1:tmax
		% Definition of the cost function for the MPC
		if tmax - iter < h
			% Horizon must be accounted for if the remaining iterations
			% are less than h
			formationPositionsH = formation(:,iter:iter + tmax-iter);
			for i = 1:h -(tmax - iter) 
				formationPositionsH = [formationPositionsH, L * formationPositionsH(:,end)];
			end
			formationPositionsH = formationPositionsH(:);
		else
			formationPositionsH = formation(:,iter:iter+h);
			formationPositionsH = formationPositionsH(:);
		end


		% Cost function
		f = (xsdp - formationPositionsH)' * Qx * (xsdp - formationPositionsH) + norm(usdp)^2;

		% Constraints for the MPC
		constraints = [xsdp == Aconstraint * xsdp + Bconstraint * usdp + [x(:,iter);zeros(nx*h,1)],...
					   -10 <= usdp, usdp <= 10];
				   
		optimize(constraints, f, S);

		u(:,iter) = value(usdp(1:nu,1));

		% update MAS state using the MPC controller
		x(:,iter+1) = A * x(:,iter) + B * u(:,iter); 

		% verify if there was a collision
		if ~isempty(obstacleCircles) && t_collision == 0
			for c = 1:size(obstacleCircles,1)
				for agent = 1:n
					agent_pos = x((((agent - 1) * 4) + 1:((agent - 1) * 4) + 2), iter + 1);
					if hasCollidedCircle(agent_pos, obstacleCircles(c,:))
						t_collision = iter;
					end
				end
			end
		end
		x_ref = x(:,iter) - formation(:,iter);
		if (iter == 1) 
			error_pos(iter) = norm(x_ref.' * kron(eye(n),diag([1 1 0 0])));
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

