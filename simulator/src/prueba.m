initPose = [0.91,1.27,-1.38]';
path = plan.path_planning(map, initPose, [3,1]);
max_w = 0.5;
max_v = 0.29;
[w_vec, v_vec] = plan.generate_odometry(path, initPose(3), 0.1, 0.5, 0.29);

vxRef = 0.05*ones(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
wRef(tVec < 5) = -0.2;
wRef(tVec >=7.5) = 0.2;
wRef(tVec >= 20) = -0.1;

vxRef(1:length(v_vec)) = v_vec;
vxRef(1:length(w_vec)) = w_vec;

viz = base.Visualizer2D;
viz.mapName = 'map';

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.

    v_cmd = vxRef(idx-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(idx-1);    %      ... para que el robot haga algo.
	% empezar con velocidades relacionadas a que mida el lidar asi no se
	% choca pero puede explorar
    
    % TO DO
        % generar velocidades para este timestep
        % A*, veo que cell sigue y pongo v y w para moverme a esa celda (al centro...)
        % aca vamos a usar una pose "mejorada" en la iteracion anterior, 
        % por las mediciones que obtuvimos del LIDAR
        % jugar con acceleraciones y momento
        % fin del TO DO
    
    % a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometria
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:scaleFactor:end);
        ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometria (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1);...
			odompose.Pose.Pose.Position.Y + initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
		% Conversion de la terna del robot a la global
        vel = base.bodyToWorld(velB,pose(:,idx-1));
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,idx));
        if simular_ruido_lidar
            % Simular ruido de lidar real (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %
    % Aca el robot ya ejecuto las velocidades comandadas y devuelve en la
    % variables ranges la medicion del lidar para ser usada.
    
    % TO DO
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,idx) ) MEJORAR la pose,
        % iterando la misma con este cloud points obtenido por el LIDAR.
		
% 		if (idx == 2)
% 			particles = pf.initialize_particles(n_particles,map);
% 			best_pose = mean(particles,1);
% 		else
% 			[particles, weights] = pf.particle_filter(particles,...
% 				dd, v_cmd, w_cmd, ranges, lidar.scanAngles, lidar.maxRange,...
% 				regen_rate, regen_spread, sampleTime, map);
% 			best_pose = weights'*particles;
% 			disp(best_pose);
% 			if(min(regen_spread) > 0.01)
% 				regen_spread = var_momentum*regen_spread;
% 			end
% 		end
	% agregar una variable que indique que se localiz� el robot, a partir
	% de cierto tiempo?? o cuando se vuelve a remuestrear la gaussiana
    % Fin del TO DO
	
	% TO DO: A*
		% Solamente si todav�a no se hizo
		% path_to_a = plan.path_planning(map, best_pose, A);
        
    % actualizar visualizacion
    viz(pose(:,idx))
% 	if(idx >= 3)
% 		delete(s1);
% 		delete(s2);
% 	end
% 	figure(1); hold on;
% 	s1 = scatter(particles(:,1),particles(:,2));
% 	s2 = scatter(best_pose(1), best_pose(2), 'x');
	hold off;
    waitfor(r);
end
