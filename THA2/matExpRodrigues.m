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
if(norm(w) == 0)
    % if norm of omega is zero, set rotation to identity.
    R = eye(3);
else
    % Rodrigue's formula
    R = eye(3) + w_hat * sin(theta) + w_hat * w_hat * (1 - cos(theta)) ;
end
end