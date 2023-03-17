 %% need to revisit and comment

function J = J_space(robot, thetaList)
T  = eye(4)
for i = 1: robot.numJoints
    if(i ~= 1)
    T  = adjoint(T * transMatExpScrew(robot.S(i)));
    end
    J(:, 1) = T * robot.S(i);
end
J= 0; % change
end