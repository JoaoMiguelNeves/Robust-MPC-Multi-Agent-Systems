% Based on Ribeiro et al. (2021)

clearvars;
close all;

global n sr collision_treshold

n = 10;                 % # of agents
t = 100;                % simulation time

% simulation variables
sr = 15;                % sensing radius
collision_treshold = 5; % treshold for collision avoidance
q = ones(2,n);          % position matrix 

theta = 0.07;
beta = 0.01;
gamma = 0.6;
delta = 0.08;
epsilon = 1.0;
zeta = 1.0;
eta = 0.01;

optimal_x = 50;
optimal_y = 75;

% Defining cost function
function res = cost (x,y)
    res = (x - optimal_x)^2 + (y - optimal_y)^2;
end

function res = neighboring_set (q_in, radius)
    res = eye(n);
    for i = 1:n
        for j = i+1:n
            if sqrt((q_in(1,j) - q_in(1,i))^2 + (q_in(2,j) - q_in(2,i))^2) <= radius
                res(i,j) = 1;
                res(j,i) = 1;
            end
        end
    end
end 
    


% Computes the separation component, which serves as a 
% naive method of collision avoidance.
% note: require a smaller set of neighbors
function s = separation (q_in, collision_set)
    s = zeros(2,n);
    for i = 1:n
        c = 1;
        for j = 1:n
            if i == j
                continue
            elseif collision_set(i,j) == 1
                s(1,i) = s(1,i) + q_in(1,i) - q_in(1,j);
                s(2,i) = s(2,i) + q_in(2,i) - q_in(2,j);
                c = c + 1;
            end
        end
        s(:,i) = s(:,i) / norm(s(:,i));
    end
end

% Computes the cohesion component, which serves as a 
% method to decrease the overall distance between neighbors,
% preventing them from separating excessively.
function s = cohesion (q_in, neighbors)
    s = zeros(2,n);
    for i = 1:n
        c = 1;
        for j = 1:n
            if i == j
                continue
            elseif neighbors(i,j) == 1
                s(1,i) = s(1,i) + q_in(1,j) - q_in(1,i);
                s(2,i) = s(2,i) + q_in(2,j) - q_in(2,i);
                c = c + 1;
            end
        end
        s(:,i) = s(:,i) / norm(s(:,i));
    end
end

% The Alignment component causes a cluster of agents to align 
% their direction vectors. Notably, in this version, the agents' 
% velocity vectors are known, while in the dissertation, they must
% be communicated by the broadcast tower.

function s = alignment (v_in, neighbors)
    s = zeros(2,n);
    for i = 1:n
        c = 1;
        for j = 1:n
            if i == j
                continue
            elseif neighbors(i,j) == 1
                s(1,i) = s(1,i) + v_in(1,j) - v_in(1,i);
                s(2,i) = s(2,i) + v_in(2,j) - v_in(2,i);
                c = c + 1;
            end
        end
        s(:,i) = s(:,i) / norm(s(:,i));
    end
end

% Attraction is responsible for moving 
% clusters to higher-utility positions.

function s = attraction (q_in, neighbors)
    s = zeros(2,n);
    for i = 1:n
        c = 1;
        for j = 1:n
            if i == j
                continue
            elseif neighbors(i,j) == 1
                cost_diff = cost(q_in(1,j), q_in(2,j)) - cost(q_in(1,i), q_in(2,i));
                s(1,i) = s(1,i) + (cost_diff * (q_in(1,j) - q_in(1,i)));
                s(2,i) = s(2,i) + (cost_diff * (q_in(2,j) - q_in(2,i)));
                c = c + 1;
            end
        end
        s(:,i) = s(:,i) / norm(s(:,i));
    end
end

% The Utility component is responsible for moving the agents 
% in the direction that locally maximizes the utility function 
% by following the function’s gradient in their position

function s = utility (q_in, neighbors)
    s = zeros(2,n);
    for i = 1:n
        c = 1;
        for j = 1:n
            if i == j
                continue
            elseif neighbors(i,j) == 1
                s(1,i) = 0;
                s(2,i) = 0;
                c = c + 1;
            end
        end
        s(:,i) = s(:,i) / norm(s(:,i));
    end
end

% The Formation component is responsible for moving the 
% agents in the direction that will ensure they arrive at 
% their proper position in the virtual formation.

function s = formation(q_in, neighbors)
    s = zeros(2,n);
    
    for i = 1:n
        c = 1;
        x_center = 0;
        y_center = 0;
        for j = 1:n
            if i == j
                x_center = x_center + q_in(1,i);
                y_center = y_center + q_in(2,i);
                continue
            elseif neighbors(i,j) == 1
                x_center = x_center + q_in(1,j);
                y_center = y_center + q_in(2,j);
                c = c + 1;
            end
        end
        x_center = x_center / c;
        y_center = y_center / c;
        formation_struct = [cos((0:c).*2*pi/n);sin((0:c).*2*pi/n)] * 6;
        formation_struct(1,:) =  formation_struct(1,:) + x_center;
        formation_struct(2,:) =  formation_struct(2,:) + y_center;
        
        s(:,i) = formation_struct(:,1);
        for k = 2:c
            form_x = s(1,i);
            form_y = s(2,i);
            form_new_x = formation_struct(1,k);
            form_new_y = formation_struct(2,k);
            if (q_in(1,i)-form_x)^2 + (q_in(2,i) - form_y)^2 > (q_in(1,i)-form_new_x)^2 + (q_in(2,i) - form_new_y)^2
                 s(:,i) = [q_in(1,i) - form_new_x;
                           q_in(2,i) - form_new_y];
            end
        end
        s(:,i) = [q_in(1,i) - s(1,i);
                  q_in(2,i) - s(2,i)];
        s(:,i) = s(:,i) / norm(s(:,i));
    end
    
end

% The Randomness component avoids deadlock situations.
function s = randomness()
    s = rand(2,n) * 2 - 1;
end