%% Robot diferencial con lidar
% Robotica Movil - 2021 2c
close all
clear all

% en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...
verMatlab = ver('MATLAB');

% simula datos no validos del lidar real, probar si se la banca
simular_ruido_lidar = false;
% false para desarrollar usando el simulador, true para conectarse al robot real
use_roomba=false;

% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress = '10.42.0.1';
    ipaddress_local = '10.42.0.123';
    setenv('ROS_IP', '10.42.0.123');
    setenv('ROS_MASTER_URI', ['http://', ipaddress, ':11311']);
    rosinit(ipaddress, 11311, 'NodeHost', ipaddress_local)
    pause(2)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(1) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

% Definicion del robot (disco de diametro = 0.35m)
robot_R = 0.35/2; % radio del robot [m]
R = 0.072/2;  % Radio de las ruedas [m]
L = 0.235;  % Distancia entre ruedas [m]
dd = base.DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

% Creacion del entorno
load ../maps/2021_2c_tp_map.mat %carga el mapa como occupancyMap en la variable 'map'

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('../pics/maps/imagen_2021_2c_mapa_tp.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release == '(R2019a)'
    disp(['Utilizando MATLAB ', verMatlab.Release]);
    imagen_mapa = 1-double(imread('../pics/maps/imagen_2021_2c_mapa_tp.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
else
    %Ni idea que pasa
    disp('ver si la compatibilidad de not R2016b, R2019a o R2020a funciona');
end

if (~use_roomba)
% Crear sensor lidar en simulador
% valores en mks
lidar = base.LidarSensor;
lidar.sensorOffset = [.09,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 2; % original: 10; %decimar lecturas de lidar acelera el algoritmo
num_scans = 144/scaleFactor; %original: 720/scaleFactor; %cambiar?
lidar.scanAngles = linspace(-pi,pi,num_scans);
lidar.maxRange = 10; % original: 8;


% Crear visualizacion
viz = base.Visualizer2D;
viz.mapName = 'map';
viz.robotRadius = robot_R;
attachLidarSensor(viz,lidar);

%%

% Celdas libres (el robot pude arrancar en cualquier lugar valido del mapa)

[free_cells_x, free_cells_y] = find(map.occupancyMatrix < map.FreeThreshold);
free_cells = [free_cells_x, free_cells_y];
n_free_cells = length(free_cells);
selected_cell = randi([1, n_free_cells]);

initPose = [grid2world(map, free_cells(selected_cell, :)), 2*pi*rand;]; % Pose inicial (x y theta) del robot simulado
%initPose = [2; 2.5; -pi/2];			% Pose inicial (x y theta) del robot simulado
%initPose = [1; 3; 0];
end
% Parametros de la Simulacion

simulationDuration = 3*60;          % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]

point_a = [3, 1];
point_b = [1.1, 2.85];

% CONV MAP
inflate_radius = robot_R * 1;
gaussian_filter_variance = 1.5;
conv_map = plan.convolve_map(map, inflate_radius, gaussian_filter_variance);
path_to_b = plan.path_planning(conv_map, point_a, point_b, map);
% ORIGINAL MAP
%path = plan.path_planning(map, initPose, goal);

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

% Inicializar matriz de pose
pose = zeros(3,numel(tVec));    
pose(:,1) = initPose;



%% Simulacion

if verMatlab.Release(1:5)=='(R201'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

% particle filter params
n_particles = 180; % cantidad de particulas
regen_rate = floor(0.1*n_particles); % cantidad de particulas que se regeneran
momentum = 0.9;
is_obs = false;
is_rot = false;
loc_th = 0.05; % localization variance of particles threshold
% flags of movement
path_planned = false;
localized = false;
go = true;
path_idx = 1; % path selection

% movement params
distance_tolerance = 0.2; % tolerancia en la distancia del robot al centro de la celda para decir que ya llego a la celda.
angle_tolerance = pi/8; % tolerancia de angulo para decir que ya esta mirando a la celda
rotate = true; % si es true, rota, si no se translada.
K_v = 2; % constante del controlador proporcional de velocidad linear
K_w = 1; % constante del controlador proporcional de velocidad angular
max_v = 0.29; % maxima velocidad lineal, satura a esta velocidad si quiere ir mas rapido.
max_w = 1; % maxima velocidad angular, satura a esta velocidad si quiere ir mas rapido.

time_running = 0;

% inicializamos las velocidades en cero.
v_cmd = 0; 
w_cmd = 0;
% 4 pasos, localizar; ir al punto A; esperar; ir al punto B.
step_n = 1;
case_name = "localization";
% tiempo ficticio de simulacion
time = 0;

%tic

% inicializamos las particulas.
particles = pf.initialize_particles(n_particles,map);
regen_spread = var(particles);
init_spread_measure = norm(regen_spread);
best_pose = mean(particles,1);

for idx = 2:numel(tVec)   
	% set movement. vcmd, wcmd
	if(localized == true)
		%best_pose = pf.sample_motion_model(dd, v_cmd, w_cmd, best_pose, sampleTime);
		if(path_planned == false)
			switch step_n
				case 1
					case_name = "Going to point A";
					path = plan.path_planning(conv_map, best_pose(1:2), point_a, map);
                    path_planned = true;
				case 2
					case_name = "Awaiting on point A";
					time_running = time_running + sampleTime;
					if (time_running >= 3)
						step_n = step_n + 1;
					end
				case 3
					case_name = "Going to point B";
					go = true;
					path = path_to_b; %plan.path_planning(conv_map, best_pose', point_b);%
                    path_planned = true;
			end
			
		end
		
		% 1 roto hasta el angulo.
		start = best_pose;
		goal = path(path_idx, :);

		x_0 = start(1);
		y_0 = start(2);
		theta_0 = start(3);

		x_1 = goal(1);
		y_1 = goal(2);

		
		if(go && localized)
			if (rotate)
				goal_angle = atan2(y_1-y_0, x_1-x_0);
				diff_angle = angdiff(goal_angle, theta_0);
				if (abs(diff_angle) > angle_tolerance)
					% ordena 
					v_cmd = 0;
					w_cmd = move.rotate(diff_angle, max_w, K_w);
				else
					rotate = false;
				end
			else
				% moverse derecho
				distance = move.euclidean_distance(start, goal);
				if ( distance > distance_tolerance)
					w_cmd = 0;
					v_cmd = move.translate(distance, max_v, K_v);
				else
					path_idx = path_idx + 1;
					if(path_idx > length(path))
						go = false;
						path_idx = 1;
						path_planned = false;
						step_n = step_n + 1;
                        if(step_n == 4)
							break;
                            % full_time = toc;
                            % display("TIME TAKEN WAS");
                            % display(full_time);
                        end
					end
					rotate = true;
				end
			end
		else % no go
			v_cmd = 0;
			w_cmd = 0;
		end
		
	end
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
    

    % Aca el robot ya ejecuto las velocidades comandadas y devuelve en la
    % variables ranges la medicion del lidar para ser usada.
	% chequeo que no tenga obstaculos
	z_t = lidar.scanAngles;
	[min_ranges, min_idx] = min(ranges);
	min_angle = pi/(length(z_t)-1);
	
	front_cone = (floor(length(z_t)/4)+1):(floor(3*length(z_t)/4));
	
	if(min(ranges(front_cone)) <= 0.25)
		is_obs = true;
		% localized = false; % para cuando agreguemos A*
		v_cmd = 0;
	else
		is_obs = false;
	end
	% IRFE (Initial Random Free Exploration)
	% si no esta localizado, ejecutar IRFE.
	
	if(localized == false)
		
		% si esta obstaculizado, que elija un sentido (horario
		% o antihorario) y rote hasta que se libere.
		if(is_obs == true)
			% si no esta rotando genera un w
			if(is_rot == false)
				% el w se genera segun los obstaculos que detecte...
				% si detecto que hay obstaculos en ambas direcciones,
				% giro aleatorio
				if(abs(z_t(min_idx)) <= min_angle)
					w_cmd = 0.75*(2*binornd(1,0.5)-1);
				% si detecto que hay obstaculos en angulos negativos,
				% entonces roto en sentido positivo
				elseif(z_t(min_idx) < -min_angle)
					w_cmd = 0.75;					
				elseif(z_t(min_idx) > min_angle)
					w_cmd = -0.75;					
				end
				is_rot = true;
				% si estaba rotando que siga rotando
			end
		% si no esta obstaculizado, que se mueva en direccion lineal
		else
			w_cmd = 0;
			is_rot = false;
			v_cmd = 0.25;
		end
	end
    
    % si esta deslocalizado se ejecuta un filtro de particulas, si esta
    % localizado se actualiza esa pose estimada con el modelo de odometría,
    % deberiamos usar EKF o algo menos exigente para seguir corrigiendo la
    % estimacion.
	
	if(go) % agregar varianza de los pesos
		% update particles
		[particles, weights] = pf.particle_filter(particles,...
			dd, v_cmd, w_cmd, ranges, lidar.scanAngles, lidar.maxRange,...
			regen_rate, regen_spread, sampleTime, map);

		best_pose = weights'*particles;
		% si regen spread (varianzas de cada coordenada) si se empiezan a clusterear accelera el algoritmo
		if(norm(regen_spread)>=init_spread_measure/10)
			regen_spread = momentum*regen_spread;
		end
		if (norm(var(particles,weights)) < loc_th)
			localized = true;
		end
	end

	%VIZ ON/OFF
	if(~use_roomba)
		% actualizar visualizacion
		viz(pose(:,idx),ranges)
		if(idx >= 3)
			delete(s1);
			delete(s2);
			delete(step_text);
			delete(time_text);
		end


		figure(1); hold on;
		s1 = scatter(particles(:,1),particles(:,2), 4, 'r', 'filled');
		hold off;

		
		figure(1); hold on;
		spointA = scatter(point_a(1), point_a(2), '*k');
		text(point_a(1)-0.2, point_a(2)-0.2, "point A");
		spointB = scatter(point_b(1), point_b(2), '*k');
		text(point_b(1)-0.2, point_b(2)-0.2, "point B");
		s2 = scatter(best_pose(1), best_pose(2), 'xk');
		s2.SizeData = 36; s2.LineWidth = 2;
		step_text = text(3, 3, "step:" + case_name);
		time_text = text(3, 2.5, "time:" + time);

		hold off;
	end

    waitfor(r);
    time = time + sampleTime;

end
