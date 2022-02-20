function [new_particles,weights] = particle_filter(particles, dd, v_cmd, w_cmd,...
	z_r, z_t, lidar_maxrange, timestep, map)
%pf.particle_filter Summary of this function goes here
%   Detailed explanation goes here

    % Initialize particles
    %particles = pf.initialize_particles(n_particles); % n_particles default 500
    final_step = 1;
	
	for t = 1:final_step
		new_particles = pf.sample_motion_model(dd, v_cmd, w_cmd, particles, timestep);
        weights = pf.measurement_model(z_r, z_t, lidar_maxrange, new_particles, map);
        normalizer = sum(weights);
        weights = weights ./ normalizer;
        particles = pf.resample(new_particles, weights);
	end
	
	% devolver particulas, calcular media y varianza afuera, pesada por los
	% weights.
	new_particles = particles;
% 	best_pose = weights'*particles;
% 	epsilon = particles-repmat(best_pose,[size(particles,1) 1]);
% 	weighted_epsilon = epsilon.*repmat(sqrt(weights),[1, size(particles,2)]);
% 	pose_confidence = diag(weighted_epsilon'*weighted_epsilon)';
    
end

