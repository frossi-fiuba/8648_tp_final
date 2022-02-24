%%
% This is the main extended Kalman filter. This script calls all the required
% functions in the correct order.
%
% You can disable the plotting or change the number of steps the filter
% runs for to ease the debugging. You should however not change the order
% or calls of any of the other lines, as it might break the framework.
%
% If you are unsure about the input and return values of functions you
% should read their documentation which tells you the expected dimensions.

function [mu, sigma] = extended_kalman_filter(old_mu, old_sigma, odo, z_r, z_t, lidar_maxRange, map)
    % Perform the prediction step of the EKF

    [mu, sigma] = ekf.prediction_step(old_mu, old_sigma, odo);

    % Perform the correction step of the EKF
    [mu, sigma] = ekf.correction_step(mu, sigma, z_r, z_t, lidar_maxRange, map);

    % Display the final state estimate
    fprintf('pose: ')
    disp('mu = '); disp(mu); disp(', sigma = '); disp(sigma);
end

