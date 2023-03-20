 %% need to revisit and comment

function Js = J_space(robot, jointAngles)

S = robot.S;

T  = eye(4);
for i = 2: robot.numJoints
   
    T  = T * transMatExpScrew(S(:, i-1), jointAngles(i-1));
    J(:, i-1) = Adj(T) * S(:, i);

end
Js = [S(:,1) J];
end