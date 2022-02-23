function v_cmd = translate(distance, max_v, K)
    
    v_cmd = K*distance;
    if (abs(v_cmd) > max_v)
        v_cmd = max_v * v_cmd / abs(v_cmd);
    end
end