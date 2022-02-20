function path = path_planning(map, start, goal)
	% start y goal tienen que ser del mundo.
	
	% mapa:
	viz = Visualizer2D;
	viz.mapName = 'map';
	viz(start);
	viz([goal,0]');
	
	% cambio de World a Grid
	start_point = start(1:2)';
	goal_point = goal;
	start = world2grid(map, start_point);
	goal = world2grid(map, goal);
	goal_y = goal(2);
	goal_x = goal(1);

	% alto y ancho del mapa
	map_size = map.GridSize;
	
	% inicializacion de costos
	costs = ones(map_size)*inf;
	
	% costo estimado al goal
	heuristics = zeros(map_size);
	speed = 3; % velocidad de la heuristica
	
	% celdas que fueron visitadas
	closed_list = zeros(map_size);
	
	% these matrices implicitly store the path
	% by containing the x and y position of the previous
	% node, respectively. Following these starting at the goal 
	% until -1 is reached returns the computed path, see at the bottom
	previous_x = zeros(map_size)-1;
	previous_y = zeros(map_size)-1;
	
	
	% iniciar busqueda en el punto de partida
	parent=start;
	costs(start(2), start(1)) = 0;
	
	% loop hasta encontrar el punto de llegada
	while(parent(1) ~= goal(1) || parent(2) ~= goal(2))
		% generar la mascara para asignar costos infinitos a celdas ya
		% visitadas
		closed_mask = closed_list;
		closed_mask(closed_mask == 1) = Inf;
		
		% encuentro los candidatos para la expansion
		open_list = costs + closed_mask + heuristics;
		
		% chequear si hay non-Inf en la open_list
		if min(open_list(:)) ==Inf
			disp('no valid path found');
			break
		end
		
		
		% encontrar las celdas con el minimo costo en la lista abierta
		[y, x] = find(open_list == min(open_list(:)));
		parent_y = y(1);
		parent_x = x(1);
		
		parent = [parent_x, parent_y];
		
		
		% pongo los parents en la lista cerrada
		closed_list(parent_y,parent_x) = 1;
		viz([grid2world(map, [parent_x, parent_y]), 0]);
		
		% neighbors de los parents
		n = neighbors([parent_y, parent_x], map_size);
		for i=1:size(n,1)
			child_y = n(i,1);
			child_x = n(i,2);
			child = [child_x, child_y];

			% calculo el costo de llegar a la celda child
			cost_val = costs(parent_y,parent_x) + edge_cost(parent, child, map);
			heuristic_val = heuristic(child, goal, speed);

			%update cost of cell
			if cost_val < costs(child_y,child_x)
			  costs(child_y,child_x) = cost_val;
			  heuristics(child_y,child_x) = heuristic_val;

			  %safe child's parent
			  previous_x(child_y,child_x) = parent_x;
			  previous_y(child_y,child_x) = parent_y;
			end
		end
		pause(0.05); % para visualizar, comentar
	end
	
	% visualization: from the goal to the start,
	% draw the path as blue dots
	parent = [goal_y, goal_x];
	distance2 = 0;
	
	j=1;
	path = zeros(prod(map_size), 2);
	path(1,:) = start;
	
	while previous_x(parent(1), parent(2))>=0
		viz([grid2world(map,[parent(2), parent(1)]),0]);
		viz([grid2world(map,goal),0]);

	% 	%for visualization: Plot goal again
	% 	if(parent(1) == goal_y && parent(2) == goal_x)
	% 		plot(grid2world(map,[goal(2), goal(1)]), 'g.');
	% 	end
		j = j+1;
		child_y = previous_y(parent(1), parent(2));
		child_x = previous_x(parent(1), parent(2));
		child = [child_y, child_x];
		path(j,:) = child;
		distance2 = distance2+norm(parent - child);
		parent = child;
		pause(0.05); %pause for visualization
	end
	
	path = path(1:distance2, :);
	disp 'done'
	disp 'path cost: ', disp(costs(goal(2),goal(1)));
	disp 'path length: ', disp(distance2);
	disp 'number of nodes visited: ', disp(sum(closed_list(:)));
end