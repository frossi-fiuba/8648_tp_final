function [w_vec, v_vec] = generate_odometry(path, theta0, time_step, max_w, max_v)

	% max vectors size
	w_vec = zeros(ceil(180/135*pi/time_step) + ceil(pi/4/time_step)*size(path,1),1);
	v_vec = zeros(size(w_vec));
	
	dif_path = (path(2:end,:) - path(1:end-1,:));
	
	theta_vec = atan(dif_path(:,2)./dif_path(:,1)); %opcion: smooth(theta_vec)
	theta_i = [theta0; theta_vec];
	% vector de w inicial
	w = (theta_vec - theta_i(1:end-1))/time_step;
	
	% vector inicial de v, una vez que el robot fue rotado
	d = vecnorm(dif_path, 2, 2);
	v = d/time_step;
	
	% check if w is > max_w and v > max_v
	oor_w_idx = find(abs(w) > max_w);
	oor_v_idx = find(abs(v) > max_v);	
	
	steps_oor_w = ceil(abs(w(oor_w_idx))/max_w);
	steps_oor_v = ceil(abs(w(oor_v_idx))/max_v);
	j=1;
	for i=1:length(w)
		ismember_w = ismember(i, oor_w_idx)*steps_oor_w(oor_w_idx==i);
		ismember_v = ismember(i, oor_v_idx)*steps_oor_v(oor_v_idx==i);
		if ismember_w ~= 0
			steps = steps_oor_w(oor_w_idx==i);
			w_vec(j:j+steps-1) = w(i)/steps;
			v_vec(j:j+steps-1) = 0;
			j = j+steps;
		else
			if(w(i) ~=0)
				w_vec(j) = w(i);
				v_vec(j) = 0;
				j=j+1;
			end
		end
		if ismember_v ~= 0
			steps = steps_oor_v(oor_v_idx==i);
			v_vec(j:j+steps-1) = v(i)/steps;
			w_vec(j:j+steps-1) = 0;
			j = j+steps;
		else
			if(v(i) ~=0)
				v_vec(j) = v(i);
				w_vec(j) = 0;
				j=j+1;
			end
		end
	end
	v_vec = v_vec(1:j);
	w_vec = w_vec(1:j);
end
