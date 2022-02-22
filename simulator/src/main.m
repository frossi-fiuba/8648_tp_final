%% Robot diferencial con lidar
% Robotica Movil - 2021 2c
close all
clear all

% en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...
verMatlab= ver('MATLAB');


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
R = 0.072/2;  % Radio de las ruedas [m]
L = 0.235;  % Distancia entre ruedas [m]
dd = base.DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

% Creacion del entorno
load ../maps/2021_2c_tp_map.mat     %carga el mapa como occupancyMap en la variable 'map'

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
attachLidarSensor(viz,lidar);

%%
% Parametros de la Simulacion

simulationDuration = 3*60;          % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [1; 1; pi/6];       % Pose inicial (x y theta) del robot simulado
%initPose = [4.5; 3.5; -pi/2]; 
% (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

% generar comandos a modo de ejemplo
vxRef = 0.05*zeros(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
% wRef(tVec < 5) = -0.2;
% wRef(tVec >=7.5) = 0.2;
% wRef(tVec >= 20) = -0.1;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

n_particles = 180;
old_best_pose = zeros(1,3);
%% Simulacion

if verMatlab.Release(1:5)=='(R201'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

regen_rate = floor(0.1*n_particles); % cantidad de particulas que se regeneran
momentum = 0.9;
localized = false;
is_obs = false;
is_rot = false;
% theta = 0;
% iterations = 0;

for idx = 2:numel(tVec)   

    % velocidades iniciales, las primeras tienen que ser nulas,
	% de esa forma evitamos que se choque con algo:
	
	if(idx == 2 && ((vxRef(idx-1) ~= 0) || (wRef(idx-1) ~= 0)))
		vxRef(idx-1) = 0;
		wRef(idx-1) = 0;
	end
	
	% adquiere velocidades para esta iteracion
    v_cmd = vxRef(idx-1);
    w_cmd = wRef(idx-1);
    
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

    % Aca el robot ya ejecuto las velocidades comandadas y devuelve en la
    % variables ranges la medicion del lidar para ser usada.
	
	% chequeo que no tenga obstaculos
	z_t = lidar.scanAngles;
	[min_ranges, min_idx] = min(ranges);
	min_angle = pi/(length(z_t)-1);
	
	front_cone = (floor(length(z_t)/4)+1):(floor(3*length(z_t)/4));
	
	if(min(ranges(front_cone)) <= 0.2)
		is_obs = true;
% 		localized = false; % para cuando agreguemos A*
		vxRef(idx) = 0;
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
					wRef(idx) = 0.75*(2*binornd(1,0.5)-1);
				% si detecto que hay obstaculos en angulos negativos,
				% entonces roto en sentido positivo
				elseif(z_t(min_idx) < -min_angle)
					wRef(idx) = 0.75;					
				elseif(z_t(min_idx) > min_angle)
					wRef(idx) = -0.75;					
				end
				is_rot = true;
				% si estaba rotando que siga rotando
			else
				wRef(idx) = wRef(idx-1);
			end
		% si no esta obstaculizado, que se mueva en direccion lineal
		else
			wRef(idx) = 0;
			is_rot = false;
			vxRef(idx) = 0.25;
		end
	end
    
    % si esta deslocalizado se ejecuta un filtro de particulas, si esta
    % localizado se actualiza esa pose estimada con el modelo de odometría,
    % deberiamos usar EKF o algo menos exigente para seguir corrigiendo la
    % estimacion.
	if (idx == 2)
        particles = pf.initialize_particles(n_particles,map);
        regen_spread = var(particles);
        init_spread_measure = norm(regen_spread);
        best_pose = mean(particles,1);
    elseif(localized == false)
        [particles, weights] = pf.particle_filter(particles,...
            dd, v_cmd, w_cmd, ranges, lidar.scanAngles, lidar.maxRange,...
            regen_rate, regen_spread, sampleTime, map);
        best_pose = weights'*particles;
        if(norm(regen_spread)>=init_spread_measure/10)
            regen_spread = momentum*regen_spread;
        end
        if(norm(var(particles,weights))<0.025)
            localized = true;
        end
	end
    
	if(localized == true)
        best_pose = pf.sample_motion_model(dd, v_cmd, w_cmd, best_pose, sampleTime);
	end
	
    % Fin del TO DO
	
	% TO DO: A*
    % Solamente si todavia no se hizo
    % path_to_a = plan.path_planning(map, best_pose, A);

    % actualizar visualizacion
    viz(pose(:,idx),ranges)
	if(idx >= 3)
		delete(s1);
		delete(s2);
	end
	figure(1); hold on;
	s1 = scatter(particles(:,1),particles(:,2), 4, 'r', 'filled');
	s2 = scatter(best_pose(1), best_pose(2), 'xk');
    s2.SizeData = 36; s2.LineWidth = 2;
	hold off;
    waitfor(r);
end

