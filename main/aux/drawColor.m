function h = drawColor(x_in,y_in)
% Utility function to draw the agent's objective.
    x=-50:1:50;
    y=-50:1:50;
    [X,Y]=meshgrid(x,y);
    Z = 100 - abs(X - x_in) - abs(Y - y_in) - 1000;
    h = surf(X,Y,Z,'edgecolor', 'none');
    view(2)
end

