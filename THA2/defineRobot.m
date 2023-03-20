function [robot, err] = defineRobot(M, S, B)
r.M = M;
r.S = S;
r.B = B;

if(size(S, 2) ~= size(B, 2))
    disp('Error: Screw axes base frame and end-effector frame size mismatch')
    robot = 0;
    err = '1';
    return
else
    r.numJoints = size(S, 2);
end

% Calculate SE(3) form for base screw axes, S
S_se_3 = eye(4);
for i=1:r.numJoints
    screw_axis = S(:, i);
    S_se_3(:, :, i) = [skew(screw_axis(1:3)) screw_axis(4:6);0 0 0 0];
end
r.S_se_3 = S_se_3; 

% Calculate SE(3) form for end effector screw axes, B
B_se_3 = eye(4);
for i=1:r.numJoints
    screw_axis = B(:, i);
    B_se_3(:, :, i) = [skew(screw_axis(1:3)) screw_axis(4:6);0 0 0 0];
end
r.B_se_3 = B_se_3; 

robot = r;
err = '0';

end
