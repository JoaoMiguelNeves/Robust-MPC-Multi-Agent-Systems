clearvars;
close all;

algorithm = 'mpc';

h = 10;		% horizon of MPC
d = 2;		% dimensions of the space where agents move (R^2, R^3, etc.)
m = 2 * d;	% mth order agent dynamics (velocity and position are both considered)

% Obstacles are circles (pos_x1, pos_x2, radius)
obstacleCircles = [];
obstacleCircles = [16 30 8; 30 18 8; 16 -20 5; 30 -12 5];
obstacleCircles = [obstacleCircles; 15 15 7];
% Load premade formation
load('formations/s_wave_5_200_8.mat', 'formation', 'n', 'tmax');
tic;
if strcmp(algorithm, 'lqr')
	[x, error, energy, distance] = lqr(n,tmax,m, formation);
elseif strcmp(algorithm, 'mpc')
	[x, error, energy, distance] = mpc_solver(n,tmax,m,h,formation,obstacleCircles);
elseif strcmp(algorithm, 'lyapunov')
	[x, error, energy, distance] = lyapunov(n,tmax,m,formation,obstacleCircles);
else
	disp(['Invalid Solver "', algorithm, '"']);
end
time = toc;
%% Write Video

% Save measurements to file
filename = algorithm + "_" + n + "_" + tmax + ".mat";
save(filename, 'x', 'error', 'energy', 'distance', 'time');

% Creating video name
filePath = matlab.desktop.editor.getActiveFilename;
filePath = strsplit(filePath,'/');

video_path = filePath(1);
for l = filePath(2:end-1)
    video_path = strcat(video_path, '/', l);
end

video_path = strcat(video_path, '/', algorithm);

video_name = video_path{1,1};

video_name = convertStringsToChars(strcat(video_name, '_', string(n), '_', string(tmax), '.avi'));

try
    close(video_name);
catch
end

create_video(video_name, x, n, m, tmax, algorithm, obstacleCircles, error, energy, distance);

 