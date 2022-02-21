function weight = measurement_model(z_r, z_theta, lidar_maxRange, x, map)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z_r: set of lidar measures (array of lengths in meters)
	% z_theta: set of lidar angles (array of angles)
    % x: set of current particles
    % map: map of the environment (OccupancyGrid object)
	
    sigma = 0.2;
    weight = ones(size(x, 1), 1);
	intersects = zeros(length(z_theta),2,size(x, 1));
	
	% genero un tensor con todas las poses x,y para calcular distancias de
	% forma optima (explicado en la pagina 7 del Jamboard)
	x_tensor = repmat(reshape(x(:,1:2)',[1,2,size(x, 1)]),[length(z_theta),1,1]);
	
	
	% esto tarda 3.7 segundos en correr, no se puede hacer matriz por la
	% funci�n en s�. No se pueden usar menos particulas porque no converge.
	for i = 1:size(x, 1)
		% sumamos pi y corremos los puntos que se tracean
		% para compensar el cambio de coordenadas entre el lidar
		% y el robot.
		lidar_offset = [0.09, 0, 0] % from the robot perspective, in meters
		%lidar_offset = [0.09*cos(x(i,3)),0.09*sin(x(i,3)),0] % in meters 
		
		intersects(:,:,i) = rayIntersection(map, x(i,:) + lidar_offset,...
			z_theta, lidar_maxRange);
		
		NaN_idx = find(~isnan(intersects(:,1,i)));

		clean_intersects = intersects(NaN_idx,:,i);
		
		clean_xtensor = x_tensor(NaN_idx,:,i);
		% https://www.mathworks.com/help/robotics/ref/binaryoccupancymap.rayintersection.html con respecto del mapa lo devuelve entonces es -clean_xtensro?
		clean_dist = sqrt(sum((clean_intersects-clean_xtensor).^2,2));
		
		% computo distancias entre poses de particulas y sus proyecciones
		%dist = sqrt(sum((intersects-x_tensor).^2,2));
		% convierto de tensor a matriz
		%dist = reshape(dist, [size(dist,1), size(dist,3), size(dist,2)]);
		
		% gaussiana (probar mezcla para ser mas realista)
		weight(i) = weight(i).*mean(normpdf(clean_dist-z_r(NaN_idx),0,sigma), 1)';
	end
end
