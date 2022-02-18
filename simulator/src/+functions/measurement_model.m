function weight = measurement_model(z, x, l)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z: set of landmark observations. Each observation contains the id of the landmark observed in z(i).id and the measured range in z(i).range.
    % x: set of current particles
    % l: map of the environment composed of all landmarks
    sigma = [0.2];
    weight = ones(size(x, 1), 1);
    x_ = x(:, 1:2); % retrieve only x and y positions, leave orieantation out

    if size(z, 2) == 0
        return
    end
    
    for i = 1:size(z, 2)
        landmark_position = [l(z(i).id).x, l(z(i).id).y];
        measurement_range = [z(i).range];
        real_distance = vecnorm((x_ - landmark_position),2,2); % mean of Normal distribution
        
        weight = weight.*normpdf(measurement_range, real_distance, sigma); % likelihood of getting the measured range  
    end
    weight = weight ./ size(z, 2);
end
