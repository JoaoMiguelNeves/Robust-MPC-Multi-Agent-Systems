function [x, t_collision] = lqr(n, tmax, m, obstacleVertices, obstacleCirlces)
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
	
	% formation definition
	v = 10*[cos((0:4).*2*pi/n);sin((0:4).*2*pi/n)] + 10;
	v = v(:)';
	fvelocity = 0.2;
	
	% use vert2lcon to obtain linear constraints for collision detection
	if ~isempty(obstacleVertices)
		[Ac,bc,~,~] = vert2lcon(obstacleVertices);
		detect_collisions = true;
	else
		detect_collisions = false;
	end

	% Define a possible time of collision to limit video
	t_collision = 0;
	
	% Define agents that are active, i.e. haven't collided
	active_agents = 1:n;
	inactive_agents = [];
	
	for iter = 1:tmax
		% Discrete-time LQR (infinite horizon without constraints) controller
		% error in the formation
		xss = [reshape(v,2,n);0.1*ones(2,n)];
		xss = xss(:);
		e = x(:,iter) - xss;
		[K,S,E] = dlqr(A,B,1000*eye(nx),1*eye(nu));
		
		% update formation
		v = fvelocity + v;

		% select actuation
		u(:,iter) = -K * e;
		
		% for agents that are inactive, ignore actuation
		if ~isempty(inactive_agents)
			u(inactive_agents,iter) = 0;
		end
		
		% update MAS state 
		x(:,iter+1) = A * x(:,iter) + B * u(:,iter);

		% check for collisions
		if detect_collisions
			for agent = active_agents
				agent_pos = x((((agent - 1) * 4) + 1:((agent - 1) * 4) + 2), iter + 1);
				if hasCollided(agent_pos, Ac, bc)
					collided_agent = active_agents == agent;
					active_agents(collided_agent) = [];
					inactive_agents = [inactive_agents, (agent-1)*2 + 1, (agent-1)*2 + 2];
					x((((agent - 1) * 4) + 3:((agent - 1) * 4) + 4), iter + 1) = 0;
				end
			end
		end

		if t_collision ~= 0 
			break
		end
	end
end

