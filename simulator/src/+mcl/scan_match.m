function [new_particles] = scan_match(particles,old_particles,measures,angles,...
	lidar_maxRange,u,maps,sampleTime,sigma_candidates,n_candidates)
	%SCAN_MATCH Summary of this function goes here
	%   Detailed explanation goes here
	
	n_particles = size(particles,2);
	
	v_cmd = u(1);
	w_cmd = u(2);
	
	new_particles = zeros(size(particles));
	candidate_particles = zeros([size(particles),n_candidates]);
	P_measure = zeros(n_candidates, n_particles);
	P_movement = zeros(n_candidates, n_particles);
	
	sigma = diag(sigma_candidates);
	sigma_movement = 0.2;
	
	for i = 1:n_particles
		% 3 x n_particles x n_candidates
		candidate_particles(:,i,1:end-1) = mvnrnd(particles(:,i),sigma,n_candidates-1)';
	end
	
	candidate_particles(:,:,end) = particles;
	
	for i = 1:n_candidates
		P_measure(i,:) = mcl.measurement_model(measures, angles, lidar_maxRange,...
			candidate_particles(:,:,i)', maps)';
		% n_candidates x n_particles
		
		teorethical_particles = old_particles + ...
		[v_cmd*cos(old_particles(3,:));
		v_cmd*sin(old_particles(3,:));
		repmat(w_cmd,1,size(particles,2))]*sampleTime;
	
		P_movement(i,:) = normpdf(sqrt(diag((candidate_particles(:,:,i)-teorethical_particles(:,:))'...
			*(candidate_particles(:,:,i)-teorethical_particles(:,:)))),0,sigma_movement);
	end
	
	p = P_measure .* P_movement;
	
	p_maximums = repmat(max(p),[n_candidates,1]);
	candidates = ~(p-p_maximums);
	[candidates_numbers, particles_numbers] = find(candidates);
	
	for i=1:n_particles
		new_particles(:,i) = candidate_particles(:,i,candidates_numbers(i));
	end
	%new_particles = candidate_particles(:,particles_numbers,candidates_numbers);
	
end

