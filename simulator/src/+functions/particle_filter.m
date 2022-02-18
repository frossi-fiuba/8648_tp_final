function [particles] = particle_filter(n_particles, u, z)
%PARTICLE_FILTER Summary of this function goes here
%   Detailed explanation goes here

    % Initialize particles
    particles = initialize_particles(n_particles); % n_particles default 500
    % CHEQUEAR QUE FINAL STEP USAMOS
    %final_step = size(data.timestep, 2);
    %final_step = 50;
    for t = 1:final_step
        new_particles = sample_motion_model(u, particles);

        weights = measurement_model(z, new_particles, map);
        normalizer = sum(weights);
        weights = weights ./ normalizer;
        particles = resample(new_particles, weights);
    end
    
end

