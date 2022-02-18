function weight = measurement_model(z, x, map)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z: set of lidar measures (LidarSensor object)
    % x: set of current particles
    % map: map of the environment (OccupancyGrid object)
	
    sigma = 0.2;
    weight = ones(size(x, 1), 1);
	intersects = zeros(length(z.scanAngles),2,size(x, 1));
	
	% genero un tensor con todas las poses x,y para calcular distancias de
	% forma optima (explicado en la pagina 7 del Jamboard)
	x_tensor = repmat(reshape(x(:,1:2)',[1,2,size(x, 1)]),[length(lidar.scanAngles),1,1]);
	
	for i = 1:size(x, 1)
		intersects(:,:,i) = map.rayIntersection(map, x(i,:), z.scanAngles, z.maxRange);
	end
	
	% computo distancias entre poses de particulas y sus proyecciones
	dist = sqrt(sum((intersects-x_tensor).^2,2));
	% convierto de tensor a matriz
	dist = reshape(dist, [size(dist,1), size(dist,3), size(dist,2)]);
    
	% pasaje de coordenadas y gaussiana
	
% 	for i = 1:size(z, 2)
%         landmark_position = [l(z(i).id).x, l(z(i).id).y];
%         measurement_range = [z(i).range];
% 		
% 		% mean of normal distribution
%         real_distance = vecnorm((x_-landmark_position),2,2);
%         
% 		% likelihood of getting the measured range
%         weight = weight.*normpdf(measurement_range, real_distance, sigma);
% 	end
	
    weight = weight ./ size(z, 2);
	
end
