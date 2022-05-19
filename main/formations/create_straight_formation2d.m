function formation = create_straight_formation2d(n, tmax, velocity_x1, velocity_x2, r)
	% Creates a formation definition to be used by the solver. 
	% Prints to a file to be used later.
	% Formation is of the form [x11, x12, v11, v12,...], corresponding to
	% position and velocity.
	%
	% This formation aligns the agents in a circle shape where the center 
	% moves in a straight line. The agents move in a 2 dimensional space.
	% n = number of agents
	% tmax = number of solver iterations
	% velocity = how quickly the formation moves
	% r = radius of the circle
	
	% define the initial positions in a circl
	formation = zeros(4 * n, tmax);
	
	% defining first column
	f = r*[cos((0:n-1).*2*pi/n);sin((0:n-1).*2*pi/n)];
	f = [reshape(f,2,n); velocity_x1*ones(1,n); velocity_x2*ones(1,n)];
	f = f(:)';
	
	formation(:,1) = f;
	
	Li = kron([1 1; 0 1],eye(2));
	L = kron(eye(n), Li);
	
	for i = 2:tmax
		formation(:,i) = L * formation(:,i-1);
	end
	
	filename = "straight_" + n + "_" + tmax + ".mat";
	
	save(filename, 'formation', 'n', 'tmax');
end

