function new_bel = markov_loc(bel, u, z, map)
%markov_loc: page 163 (174 of pdf) Probabilistc robotics
    n_cells = prod(map.GridSize);
    

    
    aux_bel = 0*bel;
    for i=1:n_cells
        aux_bel(i) = sum(sum(p_be_there_mat(i, u, map).*bel));
        new_bel(i) = eta * p_vec(z_t | x_t, m) .* aux_bel(i);
    end

    for i=1:n_cells
        for j = 1:n_cells % integrating
            bel(i) = bel(i) + p_be_there(i, j, u, map)*bel(j);
        end
    bel(i) = eta * bel(i) * p(z_t | x_t, m);
    end

end