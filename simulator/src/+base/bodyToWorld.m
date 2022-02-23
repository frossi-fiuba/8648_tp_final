function velW = bodyToWorld(velB,pose)
% BODYTOWORLD Converts a 2D velocity input velB = [vx;vy;w] from body 
% (vehicle) coordinates to world (global) coordinates.
% pose = 3xN array;
% velB = 3x1 array;
% vx = Linear velocity in x-direction (longitudinal)
% vy = Linear velocity in y-direction (lateral)
% w = Angular velocity (about z-axis)
%
% Copyright 2018 The MathWorks, Inc.

%     theta = pose(3);
%     velW = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1]*velB;

    theta = pose(3,:);
    len_theta = length(theta);
    theta = reshape(theta, 1, 1, len_theta);
    
    T = [cos(theta), -sin(theta), 0*ones(1,1,len_theta);
         sin(theta), cos(theta), 0*ones(1,1,len_theta);
         0*ones(1,1,len_theta), 0*ones(1,1,len_theta), ones(1,1,len_theta)];
          
    %velW = T*(velB.*ones(1,1,len_theta));
    
              
    velW = reshape(permute(T,[1 3 2]),3*len_theta,3)*(velB);
    velW = reshape(velW, 3, len_theta);
    

end
