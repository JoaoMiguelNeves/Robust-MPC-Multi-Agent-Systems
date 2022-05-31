function create_trajectory(x, n, obstacleCircles)
	close all;
	hold on;

	% initialize scatter plot to update later
	for i = 1:n
		plotX = x(-3 + 4*i:4*n:end,:);
		plotY = x(-2 + 4*i:4*n:end,:);

		plotX = plotX(:);
		plotY = plotY(:);
		
		c = linspace(1,i,n);
		s = plot(plotX,plotY);
		xlim(50*[-1 1]);
		ylim(50*[-1 1]);
	end
	if ~isempty(obstacleCircles)
		drawCircles(obstacleCircles);
	end
end

