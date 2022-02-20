function particles = initialize_particles(count,map)
    % Returns a set of randomly initialized particles.
    pseudo_particles = [
        unidrnd(201, count, 1), ...
        unidrnd(210, count, 1)];
	
	oor_idx = find(getOccupancy(map,pseudo_particles(:,:),"grid")>=map.FreeThreshold);
	
	for i=oor_idx'
		while(getOccupancy(map,pseudo_particles(i,:),"grid")>=map.FreeThreshold)
			pseudo_particles(i,:) =  [unidrnd(201,[1 1]), ...
									  unidrnd(210,[1 1])];
		end
	end
	
	particles = [grid2world(map, pseudo_particles(:,:)), unifrnd(-pi, pi, count, 1)];
end

