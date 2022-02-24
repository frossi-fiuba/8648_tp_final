function weight = measurement_model(z_r, z_t, lidar_maxRange, x, maps)
    % weight = measurement_model(z_r, z_t, lidar_maxRange, x, map)
    % Computes the observation likelihood of all particles.
    % The employed sensor model is range only.
    %
    % z_r: set of lidar measures (array of lengths in meters)
	% z_t: set of lidar angles (array of angles)
    % lidar_maxRange: maximum range of lidar
    % x: set of current particles
    % maps: map of the environment of each particle (OccupancyGrid object)
	
    % variance for measures comparation
    sigma = 0.2;
    % initialization of weights
    weight = zeros(size(x, 1), 1);
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
		lidar_offset = [0.09*cos(x(i,3)),0.09*sin(x(i,3)),0]; % in meters 
		
		if(or(x(i,1)+lidar_offset(1)>maps(i).XLocalLimits(2) || x(i,1)+lidar_offset(1)<maps(i).XLocalLimits(1),...
				x(i,2)+lidar_offset(2)>maps(i).YLocalLimits(2) || x(i,2)+lidar_offset(2)<maps(i).YLocalLimits(1)))
			weight(i) = 0;
			%disp("fuera");
			continue;
		elseif (getOccupancy(maps(i), x(i,1:2)) > maps(i).OccupiedThreshold)
			%disp(x(i,:));
			weight(i) = 0;
			%disp("ocupad");
			continue;
		else
			intersects(:,:,i) = rayIntersection(maps(i), x(i,:) + lidar_offset,...
			z_t, lidar_maxRange);
		end
		
        % get all indexes with non-NaN entries
		NaN_idx_intersects = find(~isnan(intersects(:,1,i)));
		NaN_idx_measures = find(~isnan(z_r));
		
		idx = NaN_idx_intersects(ismember(NaN_idx_intersects,NaN_idx_measures));
        % define a clean vector of intersections without NaNs
		clean_intersects = intersects(idx,:,i);
		
        % define a clean x tensor corresponding to the clean intersections
		clean_xtensor = x_tensor(idx,:,i);
		
        % calculate a clean distance between intersections and particle
		clean_dist = sqrt(sum((clean_intersects-clean_xtensor).^2,2));
		
		if(isempty(clean_dist) == true)
			weight(i) = 0;
		else

		% compute weight of particle using gaussian distribution
		weight(i) = mean(normpdf(clean_dist-z_r(idx),0,sigma), 1)';
		end
	end
	
end
