function [x, t_collision] = mpc_solver(n, h, tmax, m, detect_collisions, obstacleVertices)
% Solves the optimization problem using the mpc method

	Ai = kron([1 1;0 1], eye(2)); % double integrator ith node dynamics
	Bi = kron([0;1],eye(2));   

	A = kron(eye(n),Ai); % MAS dynamics
	B = kron(eye(n),Bi);
    
	nx = n*m;           % number of states of MAS
	nu = n*size(Bi,2);  % number of actuatiors of the MAS

	x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
	u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'
	x(:,1) = rand(nx,1);
	

	% formation definition
	v = 10*[cos((0:n-1).*2*pi/n);sin((0:n-1).*2*pi/n)] + 10;
	v = v(:)';
	fvelocity = 0.2;
	vinit = v;
	
	% use vert2lcon to obtain linear constraints for collision detection
	[Ac,bc,Aeqc,beqc] = vert2lcon(obstacleVertices);
	
	t_collision = 0;
	
	for iter = 1:tmax
		% Error norm for MPC
		xss = [reshape(v,2,n);fvelocity*ones(2,n)];
		xss = xss(:);
		e = x(:,iter) - xss;
		
		% Compute the actuation using the MPC
		xsdp = sdpvar(nx*(h+1),1); % define the variable for the state
		usdp = sdpvar(nu*h,1); % define the variable for the actuation
		
		% Definition of the cost function for the MPC
		Qx = 10*eye(nx*(h+1));
		formationPositionsH = vinit' + (iter:iter+h)*fvelocity;
		formationPositionsH = [reshape(formationPositionsH,2,n*(h+1));fvelocity*ones(2,n*(h+1))];
		formationPositionsH = formationPositionsH(:);
	
		f = (xsdp - formationPositionsH)' * Qx * (xsdp - formationPositionsH);

		% Constraints for the MPC
		Aconstraint = [zeros(nx,nx*(h+1));kron(eye(h),A),zeros(h*nx,nx)];
		Bconstraint = [zeros(nx,nu*h); kron(eye(h),B)];
		constraints = [xsdp == Aconstraint * xsdp + Bconstraint * usdp + [x(:,iter);zeros(nx*h,1)],...
					   norm(usdp,inf) <= 2];
				   
		S = sdpsettings('solver', 'sedumi');
		debugM = optimize(constraints, f,[]);

		u(:,iter) = value(usdp(1:nu,1));

		% update MAS state using the MPC controller
		x(:,iter+1) = A * x(:,iter) + B * u(:,iter); 
	end
end
		