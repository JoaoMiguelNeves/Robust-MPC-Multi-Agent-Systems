function collided = hasCollided(coords, A, b)
%CHECKCOLLISION Summary of this function goes here
%   Detailed explanation goes here
    
    M = A * coords;
    nRows = size(M);
    collided = false;
    for i = 1:nRows
        % if outside of any constraint, agent has not collided
        if M(i) > b(i)
            return;
        end
    end
    collided = true;
end

