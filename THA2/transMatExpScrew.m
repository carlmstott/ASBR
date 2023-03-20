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

% Week 5 - Lecture 2 slide 11
if(R ~= eye(3))
% if joint is revolute
T = [R, (eye(3) * theta + twist(1:3) * (1 - cos(theta)) + twist(1:3) * twist(1:3) * (theta - sin(theta))) * twist(4:6); 0, 0, 0, 1];
else
    %if joint is prismatic
    T = [R, twist(4:6) * theta; 0, 0, 0, 1];
end
end