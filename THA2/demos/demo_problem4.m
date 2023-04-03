%% This is a demo for the robot in Exercise 5.13 (Fig 5.23) on pg 209 of Modern Robotics by Lynch et al


clear; clc;

syms L;

M = [1 0 0 0;
     0 1 0 0;
     0 0 1 3*L;
     0 0 0 1];
 
S = sym('S', [6, 6]); 
S(:, 1) = [0; 0; 1; 0; 0; 0];

S(:, 2) = [0; 1; 0; 0; 0; 0];

S(:, 3) = [-1; 0; 0; 0; 0; 0];

S(:, 4) = [-1; 0; 0; 0; 0; L];

S(:, 5) = [-1; 0; 0; 0; 0; 2 * L];

S(:, 6) = [0; 1; 0; 0; 0; 0];

B = sym('B', [6, 6]); 
B(:, 1) = [0; 0; 1; -3 * L; 0; 0];

B(:, 2) = [0; 1; 0; 0; 0; 0];

B(:, 3) = [-1; 0; 0; 0; 0; -3 * L];

B(:, 4) = [-1; 0; 0; 0; 0; -2 * L];

B(:, 5) = [-1; 0; 0; 0; 0; -L];

B(:, 6) = [0; 1; 0; 0; 0; 0];

%we need to define a theta vector here
[robot, err] = defineRobot(M, S, B);

jointAngles = sym('theta', [robot.numJoints 1]);

[T, err] = FK_space(robot,jointAngles,1)

Js = J_space(robot,jointAngles)

Jb = J_body(robot, jointAngles)