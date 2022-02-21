function new_pose = sample_motion_model(dd, v_cmd, w_cmd, pose, timestep)
    % Samples new particle positions, based on old positions and odometry
    % using a DifferentialDrive object.
    %
    % dd: diferential drive object
    % v_cmd, w_cmd: odometry reading [linear_veloc angular_veloc]
    % pose: set of old particles
    % timestep: timestep of simulation

	new_pose = zeros(size(pose));
	% move robot with commands
	[wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
	% velocity
	[v,w] = forwardKinematics(dd,wL,wR);
    % velocities in robot's coordinates [vx;vy;w]
	velB = [v;0;w];

    for i=1:size(pose,1)
		% robot coordinates to global coordinates
		vel = base.bodyToWorld(velB,pose(i,:));
		% movement step
		new_pose(i,:) = pose(i,:) + vel'*timestep;
    end
    
    % TO DO: optimize this function, this for is not needed, its better if
    % this is matrix-wise.

end
