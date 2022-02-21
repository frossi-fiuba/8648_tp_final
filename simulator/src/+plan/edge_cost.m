function cost = edge_cost(parent, child, map)
  prob = getOccupancy(map, child, 'grid');
  thr = map.FreeThreshold;
  
  if prob <= thr
    dist = sqrt(sum((parent - child).^2));
    cost = dist + prob;
  else
     cost = inf;
  end
end