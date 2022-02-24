function [new_mu, new_sigma] = correction_step(mu, sigma, z_r, z_t, lidar_maxRange, map)
    % Updates the belief, i. e., mu and sigma, according to the sensor model
    %
    % The employed sensor model is range-only.
    %
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    % z: structure containing the landmark observations, see
    %    read_data for the format
    % l: structure containing the landmark position and ids, see
    %    read_world for the format

    % Compute the expected range measurements.
    % This corresponds to the function h.
    %expected_ranges= zeros(size(z, 2), 1);
    lidar_offset = [0.09*cos(mu(3)),0.09*sin(mu(3)), 0]; % in meters 
  
    intersects = rayIntersection(map, mu' + lidar_offset, z_t, lidar_maxRange);
	
    %intersects(:,:,i) = rayIntersection(map, x(i,:) + lidar_offset,...
	%		z_t, lidar_maxRange);
		% add pi and compensate for lidar coordinates wrt robot coordinates
		
    % get all indexes with non-NaN entries
    NaN_idx_intersects = find(~isnan(intersects(:,1)));
    NaN_idx_measures = find(~isnan(z_r));

    idx = NaN_idx_intersects(ismember(NaN_idx_intersects,NaN_idx_measures));

    % define a clean vector of intersections without NaNs
    clean_intersects = intersects(idx,:);
    clean_z = z_r(idx);
    clean_z_theta = z_t(idx);
    expected_ranges = zeros(size(clean_z, 2), 1);

    % Jacobian of h
    H = zeros(size(clean_z, 2), 3);
    % Measurements in vectorized form
    Z = zeros(size(clean_z, 2), 1);

    for i = 1:size(clean_z, 2)
        % datos del "world"
        landmark_position = clean_intersects(i, :);
        measurement_range = [clean_z(i)];
        
        % r_t
        aux = landmark_position-mu([1,2])';
        % z_hat_t
        expected_ranges(i) = sqrt(sum(aux.^2,2));
        
        % Jacobiano de h
        H(i, :) = [-(landmark_position(1)-mu(1)) / expected_ranges(i),-(landmark_position(2)-mu(2)) / expected_ranges(i) , 0];
        % distancia medida
        Z(i) = [clean_z(i)];
    end
    
    R = diag(repmat([0.5], size(clean_z, 2), 1));
    % covarianza de innovacion
    S = H*sigma*H' + R;
    % ganancia de kalman
    K = sigma*H'*inv(S);
    % media actualizada
    new_mu = mu + K*(Z - expected_ranges);
    % Covarianza actualizada
    new_sigma = (eye(size(K*H,1)) - K*H)*sigma;
end
