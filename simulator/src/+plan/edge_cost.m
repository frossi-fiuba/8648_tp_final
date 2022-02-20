function cost = edge_cost(parent, child, map)
  prob = getOccupancy(map, child, 'grid');
  thr = 0.4;
  
  if prob < thr
    dist = sqrt(sum((parent - child).^2));
    cost = dist + prob;
  else
     cost = inf;s
  end
end