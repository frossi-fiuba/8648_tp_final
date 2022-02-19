x = lidar2robot(radius, ang, coord)
% lidar2robot converts lidar measurements (radius, ang) to robot coordinate system 
% radius: 1xN vector
% angle: 1xN vector 
% output x: 2xN matrix, in cartesian coordinate system if coord = 'cartesian', in polar if coord = 'polar'

T = [cos(pi), -sin(pi), 0.09;
	 sin(pi),  cos(pi),    0;
	 0, 	0, 		       1];

if coord ='cartesian'
	x = T*[radius.*sin(ang); radius.*cos(ang); ones(1, length(ang))];
else
	x = [x_(1,:).^2 + x_(2,:).^2; atan2(x_(2,:), x_(1,:))];
end

end
