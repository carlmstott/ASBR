% brief: converts axis angle representation (w, theta) to 3x3 rotation matrix using Rodrigue's formula 
%
% params:
% w: axis unit vector
% theta: angle
%
% returns:
% R: 3x3 rotation matrix calculated using matrix exponentials coordinates
% (Rodrigue's formula) rotating about w by theta.


function R=matExpRodrigues(w,theta)
w_hat = skew(w);
if(norm(w_hat) < 1e-7)
    % if norm of omega_hat is less than 1e-7 (almost zero), set rotation to identity.
    R = eye(3);
else
    % Rodrigue's formula
    R = eye(3) + w_hat * sin(theta) + w_hat * w_hat * (1 - cos(theta)) ;
end
end