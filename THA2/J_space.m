% brief: calculates robot Jacobian in space frame
% params:
% robot: robot object consisting of the robot's kinematic properties
% defined in defineRobot.m 
% jointAngles: list of joint angles 0 to n
% returns:
% J: robot jacobian in space frame 6xn 


function Js = J_space(robot, jointAngles)

S = robot.S;

T  = eye(4);
for i = 2: robot.numJoints
   
    % Ref: Chapter 5, Modern Robotics
    T  = T * transMatExpScrew(S(:, i-1), jointAngles(i-1));
    J(:, i-1) = Adj(T) * S(:, i);

end
% prepend first column of jacobian equal to first screw axis.
Js = [S(:,1) J];
end