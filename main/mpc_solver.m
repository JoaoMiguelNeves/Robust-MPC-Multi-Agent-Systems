function [x, error_pos, energy_spent, distance] = mpc_solver(n, tmax, m, h, formation, obstacleCircles)
% Solves the optimization problem using the mpc method

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
	
	Li = kron([1 1; 0 1],eye(2));
	L = kron(eye(n), Li);
	
	V = kron(eye(n), diag([1 1 0 0]));
	
	% Skew value
	err = 0.05;
	Ei = kron([0;1], eye(2));
	E = kron(eye(n), Ei);
	
	
	% Run unobstructed MPC optimization if there isn't any previous one
	% saved, or if there are no obstacles
	
	[x_prev, t_collision, error_pos, energy_spent, distance] = mpc_unconstrained(n, tmax, m, h, formation, obstacleCircles);
	
	if t_collision == 0
		x = x_prev;
		return
	end
	S = sdpsettings('solver', 'mosek', 'cachesolvers', 1);
	S = sdpsettings(S, 'verbose', 0);
	
	% Optimization with constraints to avoid collisions
	if t_collision ~= 0
		for iter = 1:tmax
			if tmax - iter < h
				% Horizon must be accounted for if the remaining iterations
				% are less than h
				formationPositionsH = formation(:,iter:iter + tmax-iter);
				for i = 1:h -(tmax - iter) 
					formationPositionsH = [formationPositionsH, L * formationPositionsH(:,i + (tmax - iter))];
				end
				formationPositionsH = formationPositionsH(:);
			else
				formationPositionsH = formation(:,iter:iter+h);
				formationPositionsH = formationPositionsH(:);
			end
			
			% Cost function
			f = (xsdp - formationPositionsH)' * Qx * (xsdp - formationPositionsH);

			% Constraints for the MPC
			constraints = [xsdp == Aconstraint * xsdp + Bconstraint * usdp + [x(:,iter);zeros(nx*h,1)],...
						   -10 <= usdp, usdp <= 10];
			
			% Add constraints relating to the circle's collision
			if ~isempty(obstacleCircles)
				for c = 1:size(obstacleCircles,1)
					for i = 1:n
						if iter == tmax 
							break;
						elseif iter + h > tmax
							hmax = tmax - iter - 1;
						else
							hmax = h;
						end

						for aux = 1:hmax+1
							x_pos = x_prev((i-1)*4 + 1, iter + aux);
							y_pos = x_prev((i-1)*4 + 2, iter + aux);

							% Calculate the slope of the line connecting the agent and circle's
							% center, obtain the perpendicular line by the inverse of the slope
							m = -(x_pos - obstacleCircles(c,1))/(y_pos - obstacleCircles(c,2));

							b = obstacleCircles(c,2) - m * obstacleCircles(c,1);

							% calculate whether the agent is over or under the constraint line
							cons = y_pos > m * x_pos + b;

							% obtain the b value for the circle's limit
							b_aux = (obstacleCircles(c,3)+1)* sqrt(1 + m^2);

							if cons 
								b = b + b_aux;
								constraints = [constraints, xsdp((i-1)*4 +(aux-1)*(4*n)+2) >= xsdp((i-1)*4 +(aux-1)*(4*n)+1) * m + b];
							else 
								b = b - b_aux;
								constraints = [constraints, xsdp((i-1)*4 +(aux-1)*(4*n)+2) <= xsdp((i-1)*4 +(aux-1)*(4*n)+1) * m + b];
							end
						end
					end
				end
			end
			
			optimize(constraints, f, S);

			u(:,iter) = value(usdp(1:nu,1));

			% update MAS state using the MPC controller
			x(:,iter+1) = A * x(:,iter) + B * u(:,iter); 
			
			% add an error to the agent's velocity
			if err ~= 0
				random_error = -err + (2*err) * rand(n*2,1);
				x(:,iter+1) = x(:,iter+1) + E * random_error;
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
end