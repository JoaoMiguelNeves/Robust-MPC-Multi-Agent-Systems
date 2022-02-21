clearvars;
close all;

n = 5; % number of agents
h = 2; % horizon
tmax = 20; % simulation time
m = 4; % 4th order agent dynamics

Ai = kron([1 1;0 1], eye(2)); % double integrator ith node dynamics
Bi = kron([0;1],eye(2));

nx = n*m; % number of states of MAS
nu = n*size(Bi,2); % number of actuatiors of the MAS

A = kron(eye(n),Ai); % MAS dynamics
B = kron(eye(n),Bi);

x = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
u = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'
enorm = zeros(1,tmax); % error of the MAS formation ||x - v||
% different possibilities to initialize the position/velocity for the
% agents
% x(:,1) = [0.5, 1, zeros(1,2) , 2, 0.5, zeros(1,2) , 3, 3, zeros(1,2)  ,-2, 1, zeros(1,2)  ,-4,-3, zeros(1,2) ]'; 
x(:,1) = rand(nx,1);

% mpc variables
xmpc = zeros(nx,tmax); % [px1, py1, vx1, vy1, ..., pxn, pyn, vxn, vyn]'
umpc = zeros(nu,tmax); % [ax1, ay1, ..., axn, ayn]'
enormMPC = zeros(1,tmax); % error of the MAS formation ||x - v||
% different possibilities to initialize the position/velocity for the
% agents
% xmpc(:,1) = [0.5, 1, zeros(1,2) , 2, 0.5, zeros(1,2) , 3, 3, zeros(1,2)  ,-2, 1, zeros(1,2)  ,-4,-3, zeros(1,2) ]'; 
% xmpc(:,1) = rand(nx,1);
xmpc(:,1) = x(:,1);

% formation definition
v = 10*[cos((0:4).*2*pi/n);sin((0:4).*2*pi/n)] + 10;
v = v(:)';
vinit = v;
fvelocity = 0.5;

fid = figure;
writerObj = VideoWriter('plot.avi');
writerObj.FrameRate = 5;
open(writerObj);

% Gain matrix for the LQR
[K,S,E] = dlqr(A,B,10*eye(nx),1000*eye(nu));

for iter = 1:tmax
    % Error norm for LQR and MPC
    xss = [reshape(v,2,n);fvelocity*ones(2,n)];
    xss = xss(:);
    enorm(1,iter) = norm(x(:,iter)-xss);
    enormMPC(1,iter) = norm(xmpc(:,iter)-xss);
    
    % update formation
    v = fvelocity + v;
    
    % Discrete-time LQR (infinite horizon without constraints) controller
    % error in the formation
    xss = [reshape(v,2,n);fvelocity*ones(2,n)];
    xss = xss(:);
    
    % Error of the MAS using LQR
    e = x(:,iter) - xss;
    
	% select actuation
    u(:,iter) = -K * e; 
    
	% update MAS state 
	x(:,iter+1) = A * x(:,iter) + B * u(:,iter); 
    
    % Compute the actuation using the MPC
    xsdp = sdpvar(nx*(h+1),1); % define the variable for the state
    usdp = sdpvar(nu*h,1); % define the variable for the actuation
    
    % Definition of the cost function for the MPC
    Qx = 10*eye(nx*(h+1));
    formationPositionsH = vinit' + (iter:iter+h)*fvelocity;
    formationPositionsH = [reshape(formationPositionsH,2,n*(h+1));fvelocity*ones(2,n*(h+1))];
    formationPositionsH = formationPositionsH(:);
    
    f = (xsdp - formationPositionsH)' * Qx * (xsdp - formationPositionsH);
    
    % Constraints for the MPC
    Aconstraint = [zeros(nx,nx*(h+1));kron(eye(h),A),zeros(h*nx,nx)];
    Bconstraint = [zeros(nx,nu*h); kron(eye(h),B)];
    constraints = [xsdp == Aconstraint * xsdp + Bconstraint * usdp + [xmpc(:,iter);zeros(nx*h,1)],...
                   norm(usdp,inf) <= 2];
    
    S = sdpsettings('solver', 'sedumi');
    debugM = optimize(constraints, f,[]);
    
    umpc(:,iter) = value(usdp(1:nu,1));
    
    % update MAS state using the MPC controller
	xmpc(:,iter+1) = A * xmpc(:,iter) + B * umpc(:,iter); 
	
    % MAS trajectory
    subplot(2,2,1);
	s = scatter(x(1:m:end,iter),x(2:m:end,iter));
	s.LineWidth = 0.6;
	s.MarkerEdgeColor = 'b';
	s.MarkerFaceColor = [0 0.5 0.5];
	axis(50*[-1 1 -1 1]);
    
    % MAS trajectory error
    subplot(2,2,2);
    plot(enorm(1:iter));
    axis([0 tmax 0 50]);
    
    subplot(2,2,3);
	s = scatter(xmpc(1:m:end,iter),xmpc(2:m:end,iter));
	s.LineWidth = 0.6;
	s.MarkerEdgeColor = 'b';
	s.MarkerFaceColor = [0 0.5 0.5];
	axis(50*[-1 1 -1 1]);
    
    % MAS trajectory error
    subplot(2,2,4);
    plot(enormMPC(1:iter));
    axis([0 tmax 0 50]);
    
    frame = getframe(gcf);
	writeVideo(writerObj, frame);
end

hold off
close (writerObj);