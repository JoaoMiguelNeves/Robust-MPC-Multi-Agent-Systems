clearvars;
close all;

n = 5; % number of agents
h = 10; % horizon
tmax = 100; % simulation time
m = 4; % 4th order agent dynamics

% change this setting for simulation to stop on collision
detect_collisions = false;

obstacleVertices = [20 30; 30 10; 30 30; 40 10];
[k, av] = convhull(obstacleVertices);

% use vert2lcon to obtain linear constraints for collision detection
[Ac,bc,Aeqc,beqc] = vert2lcon(obstacleVertices);

Ai = kron([1 1;0 1], eye(2)); % double integrator ith node dynamics
Bi = kron([0;1],eye(2));

nx = n*m;           % number of states of MAS
nu = n*size(Bi,2);  % number of actuatiors of the MAS

A = kron(eye(n),Ai); % MAS dynamics
B = kron(eye(n),Bi);

x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'
% x(:,1) = [0.5, 1, zeros(1,2) , 2, 0.5, zeros(1,2) , 3, 3, zeros(1,2)  ,-2, 1, zeros(1,2)  ,-4,-3, zeros(1,2) ]'; 
x(:,1) = rand(nx,1);


% formation definition
v = 10*[cos((0:4).*2*pi/n);sin((0:4).*2*pi/n)] + 10;
v = v(:)';

c = 10;

% Define a possible time of collision to limit video
t_collision = 0;

%% Formation iterations

for iter = 1:tmax
    % update formation
    v = 0.2 + v;
    
    % Discrete-time LQR (infinite horizon without constraints) controller
    % error in the formation
    xss = [reshape(v,2,n);0.1*ones(2,n)];
    xss = xss(:);
    e = x(:,iter) - xss;
    [K,S,E] = dlqr(A,B,1000*eye(nx),1*eye(nu));
    
	% select actuation
    u(:,iter) = -K * e;
    
	% update MAS state 
	x(:,iter+1) = A * x(:,iter) + B * u(:,iter);
    
    % check for collisions
    if detect_collisions
        for agent = 1:n
            agent_pos = x((((agent - 1) * 4) + 1:((agent - 1) * 4) + 2), iter + 1);
            if hasCollided(agent_pos, Ac, bc)
                t_collision = iter;
                break
            end
        end
    end
    
    if t_collision ~= 0 
        break
    end
end

%% Write Video

if detect_collisions
    video_name = 'plot_collision.avi';
else
    video_name = 'plot_no_collision.avi';
end

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
surfColor = drawColor(c,c);
uistack(surfColor, 'bottom');
hold on;

% Draw obstacles
drawObstacles(obstacleVertices, k);

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

