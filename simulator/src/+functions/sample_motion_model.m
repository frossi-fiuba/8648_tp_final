function new_pose = sample_motion_model(u, pose, timestep)
    % Samples new particle positions, based on old positions and odometry.
    %
    % u: odometry reading [v w]
    % x: set of old particles

	% odometry readings
	v = u(1);
	w = u(2);
	
	% pose
	x = pose(1);
	y = pose(2);
	theta = pose(3);
	
	% variables
	dw = w*timestep;
	
    % compute new particle positions
    new_pose = pose + ...
	[v*sin(theta)/w, -v*cos(theta)/w, theta]...
	*[   cos(dw),	-sin(dw),	0;
        sin(dw),	cos(dw),	0;
        0,			0,			1]'...
	+[x-v*sin(theta)/w, y+v*cos(theta)/w, dw];
		
end
