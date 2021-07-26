function h = drawColor(x_in,y_in)
% Utility function to draw the agent's objective.
    x=-50:0.1:50;
    y=-50:0.1:50;
    [X,Y]=meshgrid(x,y);
    Z = 1./((X-x_in).^2 + (Y-y_in).^2 + 100);
    figure;
    h = surf(X,Y,Z,'edgecolor', 'none');
    view(2)
end

