function heur = heuristic(cell, goal, speed)
  
  heur = speed.*sqrt(sum((cell-goal).^2));  
end
