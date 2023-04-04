%% This script is used to test the singularity.m file 
% This uses the robot in Exercise 5.13 (Fig 5.23) on pg 209 of Modern Robotics by Lynch et al
% At the end, it plots manipulability ellipsoids for the generated singular
% configuration. In the test, the ellipsoids can be used to confirm that
% the calculated configuration is indeed singular.


clear; clc; close all;

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

[T, err] = FK_space(robot,jointAngles,false)

Js = simplify(J_space(robot,jointAngles))

Jb = simplify(J_body(robot,jointAngles))

% for singularity analysis, we use dummy Link length of L = 1
Jb = subs(Jb, L, 1);


% find singularity configurations
% Run this multiple times to get different configurations
thetaList = singularity(Jb, jointAngles);

Jb_numeric = subs(Jb, jointAngles, thetaList);

ellipsoid_plot_angular(Jb_numeric)          
ellipsoid_plot_linear(Jb_numeric)
