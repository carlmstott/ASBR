% brief: converts screw axis, S (w, v) to 4x4 homogenous transformation
% matrix to represent travel along screw axis by theta
% params:
% twist: twist vector (w, v), 6x1
% theta: angle of rotation
%
% returns:
% T: 4x4 homogenous transformation matrix to represent travel by theta along screw axis

function T=transMatExpScrew(twist,theta)

R = matExpRodrigues(twist(1:3),theta);

w = twist(1:3);
v = twist(4:6);
w_hat = skew(w);


% Week 5 - Lecture 2 slide 11
if(norm(w) == 0)
    % if joint is revolute
    T = [R, (eye(3) * theta + (1 - cos(theta)) * w_hat +  (theta - sin(theta))* w_hat * w_hat) * v; 0, 0, 0, 1];
else
    %if joint is prismatic
    T = [eye(3), v * theta; 0, 0, 0, 1];
end
end