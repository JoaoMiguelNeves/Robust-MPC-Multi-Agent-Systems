function h = drawColor(x_in,y_in)
% Utility function to draw the agent's objective.
    n_color = colormap;
    sz_c = size(n_color);
    min_color = ones(floor(sz_c(1)/2),1);
    min_color = min_color * n_color(floor(sz_c(1)/2),:);
    n_color(1:floor(sz_c(1)/2),:) = min_color;
    
    x=-50:0.1:50;
    y=-50:0.1:50;
    [X,Y]=meshgrid(x,y);
    Z = 1./((X-x_in).^2 + (Y-y_in).^2 + 100) - 10000;
    h = surf(X,Y,Z,'edgecolor', 'none');
    view(2);
    colormap(n_color);
end

