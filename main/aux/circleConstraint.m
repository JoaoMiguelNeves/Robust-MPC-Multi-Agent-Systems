function constraints = circleConstraint(constraints, circle, positions, n, h, xsdp)
	for i = 1:n
		x = positions((i-1)*4 + 1);
		y = positions((i-1)*4 + 2);
		
		% Calculate the slope of the line connecting the agent and circle's
		% center, obtain the perpendicular line by the inverse of the slope
		m = x - circle(1)/y - circle(2);
		
		b = circle(2) - m * circle(1);
		
		% calculate whether the agent is over or under the constraint line
		cons = y > m * x + b;
		
		% obtain the b value for the circle's limit
		b_aux = circle(3)* sqrt(1 + m^2);
		if cons 
			b = b + b_aux;
		else 
			b = b - b_aux;
		end
		
		
		
		for c = 1:h+1
			if cons
				constraints = [constraints, xsdp((i-1)*4 +(c-1)*n+2) >= xsdp((i-1)*4 +(c-1)*n+1) * m + b];
			else
				constraints = [constraints, xsdp((i-1)*4 +(c-1)*n+2) <= xsdp((i-1)*4 +(c-1)*n+1) * m + b];
			end
			
		end
			
	end
end

