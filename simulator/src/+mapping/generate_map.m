function [occ_matrix] = generate_map(ranges, theta, pose, map_size, grid_size)
%GENERATE_MAP Summary of this function goes here
%   Detailed explanation goes here

	occ_matrix = zeros(map_size);
	nNan_idx = ~isnan(ranges);
	ranges = ranges(nNan_idx);
	theta = theta(nNan_idx);
	
	theta_pose = pose(3);
	T = [cos(theta_pose), -sin(theta_pose), pose(1);
		 sin(theta_pose),  cos(theta_pose), pose(2);
				0		,			0	  ,    1    ];

	x = ranges .* cos(theta);
	y = ranges .* sin(theta);
	
	X_WORLD = T*[x'; y'; ones(1,size(x,1))];
	
	idx = floor(X_WORLD(1,:)/grid_size);
	idy = floor(X_WORLD(2,:)/grid_size);
	
	mx = idx - min(idx) + 1;
	my = idy - min(idy) + 1;
	
	occ_matrix(sub2ind(map_size, flip(my), mx)) = 1;
	
end

