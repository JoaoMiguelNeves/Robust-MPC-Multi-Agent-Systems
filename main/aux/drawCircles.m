function drawCircles(circle)
%DRAWCIRCLES Summary of this function goes here
%   Detailed explanation goes here
	r = circle(3);
	c = [circle(1),circle(2)];
	pos = [c-r 2*r 2*r];
	rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', [0 0 0], 'EdgeColor', [0 0 0]);
end

