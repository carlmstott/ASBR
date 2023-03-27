%% This is a demo for the robot in Exercise 4.11 (Fig 4.17) on pg 162 of Modern Robotics by Lynch et al


clear; clc;

M = [1 0 0 3;0 1 0 0;0 0 1 0;0 0 0 1];

S(:, 1) = [0; 0; 1; 0; 0; 0];

S(:, 2) = [0; 0; 0; 1; 0; 0];

S(:, 3) = [0; 0; 1; 0; -1; 0];

S(:, 4) = [0; -1; 0; -1; 0; -1];

S(:, 5) = [0; 0.707; 0.707; 0.707; -1.414; 1.414];

B(:, 1) = [0; 0; 1; 0; 3; 0];

B(:, 2) = [0; 0; 0; 1; 0; 0];

B(:, 3) = [0; 0; 1; 0; 2; 0];

B(:, 4) = [0; -1; 0; -1; 0; 2];

B(:, 5) = [0; 0.707; 0.707; 0.707; 0.707; -0.707];

%we need to define a theta vector here
[robot, err] = defineRobot(M, S, B);

jointAngles = sym('theta', [robot.numJoints 1]);

[T, jointToJointTransforms, err] = FK_space(robot,jointAngles)

Js = J_space(robot,jointAngles);
Jb = J_body(robot,jointAngles);
