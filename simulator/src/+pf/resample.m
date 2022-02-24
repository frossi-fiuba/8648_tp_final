function new_particles = resample(particles, weights, regen_rate, regen_spread, map, localized)
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
    
    dead_particles = M-size(surv_idx,1);
    if (localized)
        unif_particles = 0;
    else
        unif_particles = ceil(dead_particles/4);
    end
    norm_particles = dead_particles - unif_particles;
    mean_particles = weights'*particles;
    xlims = map.XLocalLimits;
    ylims = map.YLocalLimits;

	if(dead_particles > regen_rate)
		idx = 1:M;
		regen_index = ~(ismember(idx, surv_idx));
		new_particles(regen_index,1:2) = [
        unifrnd(xlims(1), xlims(2), unif_particles, 1), ...
        unifrnd(ylims(1), ylims(2), unif_particles, 1);
        mvnrnd(mean_particles(1:2),...
            diag(regen_spread(1:2)), norm_particles)];
        
        oor_idx = find(getOccupancy(map,new_particles(:,1:2))>=map.FreeThreshold);
        for i=oor_idx'
            while(getOccupancy(map,new_particles(i,1:2))>=map.FreeThreshold)
                new_particles(i,1:2) = [
                unifrnd(xlims(1), xlims(2), 1, 1), ...
                unifrnd(ylims(1), ylims(2), 1, 1)];
            end
        end
	end
	
end
