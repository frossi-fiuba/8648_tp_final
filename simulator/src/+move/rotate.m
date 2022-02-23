function w_cmd = rotate(diff_angle, max_w, K)

    % revisar;
    ang_speed = K*abs(diff_angle);

    clockwise = (diff_angle > 0);

    if clockwise
        w_cmd = -abs(ang_speed);
    else
        w_cmd = abs(ang_speed);
    end

    if (abs(w_cmd) > max_w)
        w_cmd = max_w * w_cmd / abs(w_cmd);
    end
end