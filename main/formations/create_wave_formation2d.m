function formation = create_wave_formation2d(n, tmax, velocity_x1, velocity_x2, r)
	formation = zeros(4 * n, tmax);
	%x = r*[cos((1:n).*2*pi/n);sin((1:n).*2*pi/n)];
	
	x = r*[cos((0:n-1).*2*pi/n);sin((0:n-1).*2*pi/n)];
	x = [reshape(x,2,n); zeros(2,n)];
	x = x(:)';
	formation(:,1) = x;
	v = [velocity_x1 * ones(1,tmax); velocity_x2 * sin((1:tmax)*pi/20)];
	
	Li = kron([1 1; 0 0],eye(2));
	L = kron(eye(n), Li);
	
	for i = 0:n-1
		formation(3+(i*4):4+(i*4),:) = v;
	end
	
	for i = 2:tmax
		formation(:,i) = formation(:,i) + L * formation(:,i-1);
	end
	
	
	filename = "wave_" + n + "_" + tmax + "_" + r  + ".mat";
	
	save(filename, 'formation', 'n', 'tmax');
end

