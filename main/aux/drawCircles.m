function drawCircles(circle)
%DRAWCIRCLES Summary of this function goes here
%   Detailed explanation goes here
	hold on;
	for i = 1:size(circle,1)
		r = circle(i,3);
		c = [circle(i,1),circle(i,2)];
		pos = [c-r 2*r 2*r];
		rectangle('Position',pos,'Curvature',[1 1], 'FaceColor', [0 0 0], 'EdgeColor', [0 0 0]);
	end
end

