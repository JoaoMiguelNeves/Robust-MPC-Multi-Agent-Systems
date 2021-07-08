function updateColor(h_in, x_in, y_in)
%UPDATECOLOR Summary of this function goes here
%   Detailed explanation goes here
    x=-50:1:50;
    y=-50:1:50;
    [X,Y]=meshgrid(x,y);
    Zaux = 100 - abs(X - x_in) - abs(Y - y_in) - 1000;
    set(h_in, 'ZData', Zaux);
end

