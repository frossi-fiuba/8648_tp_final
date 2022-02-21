function new_particles = resample(particles, weights, regen_rate, regen_spread)
    % Returns a new set of particles obtained by performing
    % stochastic universal sampling.
    %
    % particles (M x D): set of M particles to sample from. Each row contains a state hypothesis of dimension D.
    % weights (M x 1): weights of the particles. Each row contains a weight.
	% regen_rate (integer): amount of unique particles needed to start regeneration
	% regen_spread (x_var, y_var, theta_var): how far are the new particles generated
	
    new_particles = zeros(size(particles));
    M = size(particles, 1);
    c = zeros(size(weights,1),1);
    c(1,1) = weights(1);
    
    for i = 2:M % portion of "roulette" for each weight w(i)
        c(i,1) = c(i-1,1) + weights(i,1);
    end
    
    u = unifrnd(0, 1/M); % sample of normal distribution: initializing the "roulette"
    i = 1;
    
	for j=1:M
        while(u > c(i,1)) 
            i = i+1; 
        end
        new_particles(j,:) = particles(i,:); % new particle is chosen according to where u landed in the "roulette"
        u = u + 1/M; % incrementing u in 1/M
	end
	
	
	[~, surv_idx] = unique(new_particles, 'row');

	if(size(surv_idx,1) < regen_rate)
		idx = 1:M;
		regen_index = ~(ismember(idx, surv_idx));
		new_particles(regen_index,:) = mvnrnd(weights'*particles, diag(regen_spread), M-length(surv_idx));
	end
	
end
