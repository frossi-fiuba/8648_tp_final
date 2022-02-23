function [particles, map] = initialize_particles(count, ranges, angles, max_range, x_map_size, y_map_size, resolution)
    % Returns a set of randomly initialized particles.

	mu = [x_map_size; y_map_size]/resolution/2;
	sigma = diag([x_map_size, y_map_size])/5/resolution;

    particles = [mvnrnd(mu, sigma, count)'; unifrnd(-pi, pi, 1, count)];
	
	map = occupancyMap.empty(count,0);
	
	for i = 1:count
		map(i) = occupancyMap(0.5*ones(x_map_size,y_map_size), resolution);
		move(map(i), [particles(1,i),particles(2,i)]-mu');
		insertRay(map(i), particles(:,i), lidarScan(ranges, angles), max_range);
	end

end

