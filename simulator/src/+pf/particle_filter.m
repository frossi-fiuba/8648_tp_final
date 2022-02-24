function [new_particles, weights] = particle_filter(particles, dd, v_cmd, w_cmd,...
	z_r, z_t, lidar_maxrange, regen_rate, regen_spread, timestep, map, localized)
%[new_particles, weights]=pf.particle_filter(particles, dd, v_cmd, w_cmd,
%   z_r, z_t, lidar_maxrange, regen_rate, regen_spread, timestep, map)
%
%   This function process a step of particle filter, which has to be
%   initialized outside, by initialize_particles. Input variables are as
%   follow:
%       particles: particles to be filtered, Nx3, where N is number of
%       particles.
%       dd: DifferentialDrive object.
%       v_cmd, w_cmd: linear and angular velocity commands to execute.
%       z_r, z_t: lidar measures and its corresponding angles
%       lidar_maxrange: max range of the lidar
%       regen_rate: amount of particles that need to have survived for the
%       regeneration to start
%       regen_spread: the maximum deviation for the regenerated particles
%       timestep: time step for the simulation
%       map: map as OccuppancyGrid object

    % start by moving the particles with the motion of the robot
    new_particles = pf.sample_motion_model(dd, v_cmd, w_cmd, particles, timestep);
    % compute the weights of the particles using the lidar measurement mdl
    weights = pf.measurement_model(z_r, z_t, lidar_maxrange, new_particles, map);
    % detect NaN weights, if so they are discarded (weight set to zero)
	weights(isnan(weights)) = 0;
	% normalize the weights
    normalizer = sum(weights);
    weights = weights ./ normalizer;
    % resample particles
    new_particles = pf.resample(new_particles, weights, regen_rate, regen_spread, map, localized);
    
end

