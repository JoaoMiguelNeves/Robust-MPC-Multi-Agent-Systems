clearvars;
close all;

algorithm = 'mpc';

accepted_algorithms = ['lqr', 'mpc'];

if strcmp(algorithm, accepted_algorithms)
    return
end


n = 5; % number of agents
h = 10; % horizon
tmax = 100; % simulation time
m = 4; % 4th order agent dynamics

obstacleVertices = [];
obstacleCircles = [];
obstacleCircles = [30 20 5];

if ~isempty(obstacleVertices)
	[k, av] = convhull(obstacleVertices);
end

if strcmp(algorithm, 'lqr')
	[x, t_collision] = lqr(n,tmax,m,obstacleVertices,obstacleCirles);
elseif strcmp(algorithm, 'mpc')
	[x, t_collision] = mpc_solver(n,h,tmax,m,obstacleVertices,obstacleCircles);
end

%% Write Video

filePath = matlab.desktop.editor.getActiveFilename;
filePath = strsplit(filePath,'/');

video_path = filePath(1);
for l = filePath(2:end-1)
    video_path = strcat(video_path, '/', l);
end

video_path = strcat(video_path, '/', algorithm);

video_name = video_path{1,1};

video_name = strcat(video_name, '_collision.avi');

try
    close(video_name);
catch
end

writerObj = VideoWriter(video_name);
writerObj.FrameRate = 60;
open(writerObj);

% initialize scatter plot to update later
plotX = x(1:m:end,1);
plotY = x(2:m:end,1);

% Draw color map first
c = 10;
surfColor = drawColor(c,c);
uistack(surfColor, 'bottom');
hold on;

% Draw obstacles
if ~isempty(obstacleVertices)
	drawObstacles(obstacleVertices, k);
end

if ~isempty(obstacleCircles)
	drawCircles(obstacleCircles);
end

% Draw agents
s = scatter(plotX, plotY);
s.LineWidth = 0.6;
s.MarkerEdgeColor = 'b';
s.MarkerFaceColor = 'b';
xlim(50*[-1 1]);
ylim(50*[-1 1]);

if t_collision == 0
    video_size = tmax;
else
    video_size = t_collision;
end

for iter = 1:video_size
    plotX = x(1:m:end,iter);
    plotY = x(2:m:end,iter);
    
    set(s, 'XData', plotX, 'YData', plotY);
    
	frame = getframe(gcf);
	writeVideo(writerObj, frame);
end

 
close (writerObj);
