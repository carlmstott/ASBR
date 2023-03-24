% brief: calculates robot Jacobian in end-effector frame
% params:
% robot: robot object consisting of the robot's kinematic properties
% defined in defineRobot.m 
% jointAngles: list of joint angles 0 to n
% returns:
% J: robot jacobian in end-effector frame 6xn 


function Js = J_body(robot, jointAngles)

B = robot.B;

Jb = B;

T  = eye(4);

for i = (robot.numJoints -1) : -1 : 1
   i
    % Ref: Chapter 5, Modern Robotics
    T  = T * transMatExpScrew(-1 * B(:, i + 1), jointAngles(i + 1))
    Jb(:, i) = Adj(T) * B(:, i)

end
end