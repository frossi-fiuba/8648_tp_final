%% Robot diferencial con lidar
% Robotica Movil - 2021 2c
close all
clear all

verMatlab= ver('MATLAB');   % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

simular_ruido_lidar = true; %simula datos no validos del lidar real, probar si se la banca
use_roomba=false;  % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
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
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
load ../maps/2021_2c_tp_map.mat     %carga el mapa como occupancyMap en la variable 'map'

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('../pics/maps/imagen_2021_2c_mapa_tp.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa
    disp('ver si la compatibilidad de not R2016b, R2019a o R2020a funciona');
elseif verMatlab.Release == '(R2019a)'
    disp(['Utilizando MATLAB ', verMatlab.Release]);
    imagen_mapa = 1-double(imread('../pics/maps/imagen_2021_2c_mapa_tp.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
end

%% Crear sensor lidar en simulador
%% valores en mks
lidar = LidarSensor;
lidar.sensorOffset = [.09,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 1; % original: 10;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 144/scaleFactor; %original: 720/scaleFactor; %cambiar?
lidar.scanAngles = linspace(-pi,pi,num_scans);
lidar.maxRange = 10; % original: 8;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 3*60;          % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [2; 2.5; -pi/2];         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
vxRef = 0.05*ones(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
wRef(tVec < 5) = -0.2;
wRef(tVec >=7.5) = 0.2;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion

if verMatlab.Release(1:5)=='(R201'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end


%% belief
% vamos a tener que cambiar el belief porque existen beliefs para cada
% angulo de pose, que no tuvimos en cuenta. ese angulo tendra que ser
% considerado desde -pi a pi con un paso igual a la cantidad de puntos del
% LIDAR del robot. la forma matricial seria tener un tensor en el que cada
% capa es un angulo distinto para todas las posiciones posibles del robot.
%
% bel(:,:,1) correspondiente al angulo -pi para celdas...
%       /                                                   \
%      | bel(celda[1 1]) bel(celda[1 2]) ... bel(celda[1 n]) |
%      |       ...            ...                 ...        |
%      | bel(celda[m 1]) bel(celda[m 2]) ... bel(celda[m n]) |
%       \                                                   /
%
% bel(:,:,2) correspondiente al angulo -pi + step_LIDAR para celdas...
%       /                                                   \
%      | bel(celda[1 1]) bel(celda[1 2]) ... bel(celda[1 n]) |
%      |       ...            ...                 ...        |
%      | bel(celda[m 1]) bel(celda[m 2]) ... bel(celda[m n]) |
%       \                                                   /
%
% y asi sigue...

% celdas libres como 1, celdas ocupadas como 0.
bel = (map.occupancyMatrix <= map.FreeThreshold);

% esto lo repetimos por la resolucion del LIDAR
% bel = repmat(bel, [1 1 LIDAR_resolution]);

n_freecells = sum(sum(bel)); % n_freecells = sum(sum(sum(bel,3),2),1);

% inicializamos los valores de belief del robot en las celdas libres,
% como una uniforme de 1/n siendo n la cantidad de celdas libres
bel = bel / n_freecells; 

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.

    v_cmd = vxRef(idx-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(idx-1);    %      ... para que el robot haga algo.
    
    %% TO DO
        % generar velocidades para este timestep
        % A*, veo que cell sigue y pongo v y w para moverme a esa celda (al centro...)
        % aca vamos a usar una pose "mejorada" en la iteracion anterior, 
        % por las mediciones que obtuvimos del LIDAR
        % jugar con acceleraciones y momento
        % fin del TO DO
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:scaleFactor:end);
        ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y + initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Conversion de la terna del robot a la global
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
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variables ranges la medicion del lidar para ser usada.
    
    %% TO DO
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,idx) ) MEJORAR la pose,
        % iterando la misma con este cloud points obtenido por el LIDAR.
        % 

        % Update belief mediante markov localization, if localized = false
        % lo haremos aca pq necesitamos las mediciones del LIDAR.
        % bel = markov_loc(bel, z);
        % if var(belief) > threshold then localized = true y nunca vuelve a
        % localizarse a menos que se vuelva a perder.
        
        % Fin del TO DO
        
    %%
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
end
