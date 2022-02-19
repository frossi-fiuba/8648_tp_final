function new_bel = markov_loc(bel, z, map)
% new_belief = markov_loc(belief, control, measure, map): page 163 (174 of pdf)
% Probabilistic Robotics. THRUN, BURGARD, FOX.
% this function attempts to localize the robot while it is stationary.

    eta = 0.5;

    n_cells = prod(map.GridSize);
    
    aux_bel = 0*bel;
    new_bel = 0*bel;
    
    for i=1:n_cells
        % this is commented so the localization is stationary, without the
        % robot moving. In case localization fails, advanced_markov_loc (WIP)
        % must be used, making the robot explore to get more measurements
        % with LIDAR in another position (use belief of this function's 
        % output as bel_0 for advanced_markov_loc).
        
        % aux_bel(i) = sum(sum(p_be_there_mat(i, u, map).*bel));
        new_bel(i) = eta * p_vec(z, i, map) .* bel(i);
        % make this matrix-wise bc it WILL be a disaster.
    end

%     for i=1:n_cells
%         for j = 1:n_cells % integrating
%             bel(i) = bel(i) + p_be_there(i, j, u, map)*bel(j);
%         end
%     bel(i) = eta * bel(i) * p(z_t | x_t, m);
%     end

end