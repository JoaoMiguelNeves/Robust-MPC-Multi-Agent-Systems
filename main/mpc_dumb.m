function x = mpc_dumb(n, h, tmax, m, obstacleVertices, obstacleCircles)
% Solves the optimization problem using the mpc method

	Ai = kron([1 1;0 1], eye(2)); % double integrator ith node dynamics
	Bi = kron([1;1],eye(2));      % actuation affects both current velocity *and* current position

	A = kron(eye(n),Ai); % MAS dynamics
	B = kron(eye(n),Bi);
    
	nx = n*m;           % number of states of MAS
	nu = n*size(Bi,2);  % number of actuatiors of the MAS

	x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
	u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'	

	% formation definition
	v = 10*[cos((0:n-1).*2*pi/n);sin((0:n-1).*2*pi/n)] + 10;
	v = v(:)';
	fvelocity = 0.2;
	vinit = v;

	% Error norm for MPC
	xss = [reshape(v,2,n);fvelocity*ones(2,n)];
	xss = xss(:);

	xsdp = sdpvar(nx,1); % define the variable for the state
	usdp = sdpvar(nu,1); % define the variable for the actuation
	
	Qx = 10*eye(nx);

	for iter = 1:tmax
		% Definition of the cost function for the MPC
		formationPositionsH = vinit' + iter*fvelocity;
		formationPositionsH = [reshape(formationPositionsH,2,n);fvelocity*ones(2,n)];
		formationPositionsH = formationPositionsH(:);

		f = (xsdp - formationPositionsH)' * Qx * (xsdp - formationPositionsH);

		% Constraints for the MPC
		constraints = [xsdp == A * x(:,iter) + B * usdp, norm(usdp,inf) <= 1];

		S = sdpsettings('solver', 'mosek', 'cachesolvers', 1);
		S = sdpsettings(S, 'verbose', 0);
		optimize(constraints, f, S);


		u(:,iter) = value(usdp);

		% update MAS state using the MPC controller
		x(:,iter+1) = A * x(:,iter) + B * u(:,iter); 
	end
end