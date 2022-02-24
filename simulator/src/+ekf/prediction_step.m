function [new_mu, new_sigma] = prediction_step(mu, sigma, odo)
    % Updates the belief, i. e., mu and sigma, according to the motion model
    %
    % u: odometry reading (r1, t, r2)
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution

    % Compute the noise-free motion. This corresponds to the function g, evaluated
    % at the state mu.
    t = odo(1);
    r = odo(3);
    % nueva media es evaluar en g
    new_mu = zeros(3,1);
    
    new_mu(1) = mu(1) + t * cos(mu(3) + r);
    new_mu(2) = mu(2) + t * sin(mu(3) + r);
    new_mu(3) = mu(3) + r;
    
    % Compute the Jacobian of g with respect to the state
    G = [1, 0, -t*sin(mu(3) + r);
            0, 1, t*cos(mu(3) + r);
            0, 0, 1];

    % Motion noise
    Q = [0, 0, 0; 
        0, 0, 0; 
        0, 0, 0];
    
    % Compute the Jacobian of g with respect to the control
    V = [-t*sin(mu(3) + r), cos(mu(3) + r), 0;
           t*cos(mu(3) + r), sin(mu(3) + r), 0;
           1, 0, 1];

    % new sigma
    new_sigma = G*sigma*G' + V*Q*V';
end
