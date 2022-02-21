function particles = initialize_particles(count,map)
    % Returns a set of randomly initialized particles.

	x_map_size = map.GridSize(1)
	y_map_size = map.GridSize(2)

    pseudo_particles = [
        unidrnd(x_map_size, count, 1), ...
        unidrnd(y_map_size, count, 1)];
	
	oor_idx = find(getOccupancy(map,pseudo_particles(:,:),"grid")>=map.FreeThreshold);
	
	for i=oor_idx'
		while(getOccupancy(map,pseudo_particles(i,:),"grid")>=map.FreeThreshold)
			pseudo_particles(i,:) =  [unidrnd(x_map_size,[1 1]), ...
									  unidrnd(y_map_size,[1 1])];
		end
	end
	
	particles = [grid2world(map, pseudo_particles(:,:)), unifrnd(-pi, pi, count, 1)];
end

