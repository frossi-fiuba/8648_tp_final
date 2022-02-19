function new_pose = sample_motion_model(dd, v_cmd, w_cmd, pose, timestep)
    % Samples new particle positions, based on old positions and odometry.
    %
    % u: odometry reading [v w]
    % x: set of old particles

% 	% odometry readings
% 	v = u(1);
% 	w = u(2);
% 	
% 	% pose
% 	x = pose(:,1);
% 	y = pose(:,2);
% 	t = pose(:,3);

% 	% variables
% 	dw = w*timestep;

	new_pose = zeros(size(pose));

	% Mover el robot segun los comandos generados
	[wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
	% Velocidad resultante
	[v,w] = forwardKinematics(dd,wL,wR);
	velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]

	for i=1:size(pose,1)
		% Conversion de la terna del robot a la global
		vel = bodyToWorld(velB,pose(i,:));
		% Realizar un paso de integracion
		new_pose(i,:) = pose(i,:) + vel'*timestep;
	end
	
%	% compute new particle positions
% 	if (w ~= 0)
% 		new_pose = pose + ...
% 		[v*sin(t)/w, -v*cos(t)/w, t]...
% 		*[   cos(dw),	-sin(dw),	0;
% 			sin(dw),	cos(dw),	0;
% 			0,			0,			1]'...
% 		+[x-v*sin(t)/w, y+v*cos(t)/w, repmat(dw,[size(x,1) 1])];
% 	else
% 		new_pose = pose...
% 			+[timestep*v*cos(t),timestep*v*sin(t),zeros(size(t,1),1)];
% 	end

end
