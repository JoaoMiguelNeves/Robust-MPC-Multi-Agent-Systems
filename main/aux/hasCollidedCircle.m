function collided = hasCollidedCircle(coords, circle)
%CHECKCOLLISION Summary of this function goes here
%   Detailed explanation goes here
	x = coords(1) - circle(1);
	y = coords(2) - circle(2);
	r = circle(3);
    if (x^2 + y^2 < r^2)
		collided = true;
	else 
		collided = false;
	end
end

