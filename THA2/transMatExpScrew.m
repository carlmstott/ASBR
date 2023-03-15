% brief: converts screw axis, S (w, v) to 4x4 homogenous transformation
% matrix to represent travel along screw axis by theta
% params:
% S: Screw axis (w, v), 6x1 vector
% theta: angle of rotation
%
% returns:
% T: 4x4 homogenous transformation matrix to represent travel along screw axis

function T=transMatExpScrew(S,theta)

R = matExpRodrigues(S.w,theta);

% Week 5 - Lecture 2 slide 11
if(R ~= eye(3))
% if joint is revolute
T = [R, (eye(3) * theta + S.w * (1 - cos(theta)) + S.w * S.w * (theta - sin(theta))) * S.v; 0, 0, 0, 1];
else
    %if joint is prismatic
    T = [R, S.v * theta; 0, 0, 0, 1];
end
end