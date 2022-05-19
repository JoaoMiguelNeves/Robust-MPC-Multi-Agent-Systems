function create_video(video_name, x, n, m, tmax, algorithm, obstacleCircles,  error, energy, distance)
% Draw a video from the formation movement
writerObj = VideoWriter(video_name);
writerObj.FrameRate = 30;
open(writerObj);

% initialize scatter plot to update later
plotX = x(1:m:end,1);
plotY = x(2:m:end,1);

subplot(2,2,1);
hold on;


% Draw agents
s = scatter(plotX, plotY);
s.LineWidth = 0.6;
s.MarkerEdgeColor = 'b';
s.MarkerFaceColor = 'b';

% Plot limits

video_size = tmax;
error_limit = max(error);

for iter = 1:video_size
	% Plot the formation
	subplot(2,2,1);
	xlim(50*[-1 1]);
	ylim(50*[-1 1]);
	if ~isempty(obstacleCircles)
		drawCircles(obstacleCircles);
	end
    plotX = x(1:m:end,iter);
    plotY = x(2:m:end,iter);
    
    set(s, 'XData', plotX, 'YData', plotY);
	title('Formation');
	
	% Error subplot
	subplot(2,2,2);
    plot(error(1:iter));
	xlim([0 tmax]);
	ylim([0 (error_limit + 1)]);
	title('Error');
	
	% Energy subplot
	subplot(2,2,3);
    plot(energy(1:iter));
	xlim([0 tmax]);
	ylim([0 (energy(end) + 1)]);
	title('Cumulative Energy Spent');
	
	% Distance subplot
	subplot(2,2,4);
    plot(distance(1:iter));
	xlim([0 tmax]);
	ylim([0 (distance(end) + 1)]);
	title('Cumulative Distance Travelled');
    
	frame = getframe(gcf);
	writeVideo(writerObj, frame);
end

close (writerObj);
end

