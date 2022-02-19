function n = neighbors(cell, map_dimensions)
  n_size = 3;
  n_aux = [ [-1.*ones(1,n_size); -1:1]'; 
            [0.*ones(1,n_size-1);[-1,1]]'; 
            [1.*ones(1,n_size); -1:1]'];
  n = cell.*ones(n_size*n_size - 1, 2) + n_aux;

  pos_x = cell(2);
  pos_y = cell(1);
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);
  
  if pos_y == 1
      n = n(4:end, :); % removes the cells above the cell
  end
  if pos_y == size_y
      n = n(1:end-3, :); % removes the ones under the cell
  end
  if pos_x == 1
      [row, ~] = find(n(:, 2) ~= pos_x - 1); % removes everything that's left to the cell
      n = n(row ,:);
  end
  if pos_x == size_x
      [row, ~] = find(n(:, 2) ~= pos_x + 1); % removes everything that's right to the cell
      n = n(row ,:);
  end
end