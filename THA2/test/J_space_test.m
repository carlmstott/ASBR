%% This is a demo for a RRRP manipulator as shown in Fig 5.7 on pg 181 of Modern Robotics by Lynch et al
%% This is used to test the J_space function.

clear all; clc;

syms L1 L2

M = eye(4); % M is not used in the calculation of Jacobian. Thus set to identity.
 
S = [0 0    0       0;
     0 0    0       0;
     1 1    1       0;
     0 0    0       0;
     0 -L1 -L1-L2   0;
     0 0    0       1;];
 
 B = S; % as we are calculating the space jacobian, body screw axes are of no use. 
 
robot = defineRobot(M, S, B);

jointAngles = sym('theta', [robot.numJoints 1]);

J = J_space(robot,jointAngles)


%% The answer below corresponds with the Jacobian calculated in the book on pg 181.
% J =
%  
% [ 0,               0,                                          0, 0]
% [ 0,               0,                                          0, 0]
% [ 1,               1,                                          1, 0]
% [ 0,  L1*sin(theta1),   L2*sin(theta1 + theta2) + L1*sin(theta1), 0]
% [ 0, -L1*cos(theta1), - L2*cos(theta1 + theta2) - L1*cos(theta1), 0]
% [ 0,               0,                                          0, 1]
 