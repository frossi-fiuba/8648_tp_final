function [v_cmd, w_cmd] = move_to_point(start, goal)
    
    max_v = 0.3;
    max_w = 0.5;
   
    K_lin = 1;
    K_ang = 1;

    x_0 = start(1);
    y_0 = start(2);
    theta_0 = start(3);

    x_1 = goal(1);
    y_1 = goal(2);

    distance = plan.euclidean_distance(start, goal);

    steering_angle = atan2(y_1-y_0, x_1-x_0);

    v_cmd = K_lin*distance;
    w_cmd = K_ang*(steering_angle - theta_0);

    if (abs(v_cmd) > max_v)
        v_cmd = max_v * v_cmd / abs(v_cmd);
    end

    if (abs(w_cmd) > max_w)
        w_cmd = max_w * w_cmd / abs(w_cmd);
    end

end
