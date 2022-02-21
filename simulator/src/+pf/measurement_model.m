function weight = measurement_model(z_r, z_t, lidar_maxRange, x, map)
    % weight = measurement_model(z_r, z_t, lidar_maxRange, x, map)
    % Computes the observation likelihood of all particles.
    % The employed sensor model is range only.
    %
    % z_r: set of lidar measures (array of lengths in meters)
	% z_t: set of lidar angles (array of angles)
    % lidar_maxRange: maximum range of lidar
    % x: set of current particles
    % map: map of the environment (OccupancyGrid object)
	
    % variance for measures comparation
    sigma = 0.2;
    % initialization of weights
    weight = ones(size(x, 1), 1);
    % intialization of ray projection
	intersects = zeros(length(z_t),2,size(x, 1));
	
	% generate a tensor that contains all positions [x,y] to compute
	% all distances optimally
	x_tensor = repmat(reshape(x(:,1:2)',[1,2,size(x, 1)]),[length(z_t),1,1]);
	
	% process the weights using the intersections for each particle,
	% checking for NaN measures.
	for i = 1:size(x, 1)
		% sumamos pi y corremos los puntos que se tracean
		% para compensar el cambio de coordenadas entre el lidar
		% y el robot.
		%lidar_offset = [0.09, 0, 0] % from the robot perspective, in meters
		lidar_offset = [0.09*cos(x(i,3)),0.09*sin(x(i,3)),0] % in meters 
		
		intersects(:,:,i) = rayIntersection(map, x(i,:) + lidar_offset,...
			z_theta, lidar_maxRange);
		% add pi and compensate for lidar coordinates wrt robot coordinates
		
        % get all indexes with non-NaN entries
		NaN_idx = find(~isnan(intersects(:,1,i)));

        % define a clean vector of intersections without NaNs
		clean_intersects = intersects(NaN_idx,:,i);
		
        % define a clean x tensor corresponding to the clean intersections
		clean_xtensor = x_tensor(NaN_idx,:,i);
		% https://www.mathworks.com/help/robotics/ref/binaryoccupancymap.rayintersection.html con respecto del mapa lo devuelve entonces es -clean_xtensro?
		
        % calculate a clean distance between intersections and particle
		clean_dist = sqrt(sum((clean_intersects-clean_xtensor).^2,2));
		
		% compute weight of particle using gaussian distribution
		weight(i) = weight(i).*mean(normpdf(clean_dist-z_r(NaN_idx),0,sigma), 1)';
        % TO DO: test other, more realistic distributions, like mixtures?
	end
end
