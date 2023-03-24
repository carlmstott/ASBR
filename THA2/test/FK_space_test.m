%% This is a demo for a RRR planar manipulator as shown in Fig 4.1 on pg 138 of Modern Robotics by Lynch et al
%% This is used to test the FK_space function.

clear all; clc;

L = sym('L', [3 1]);

M = [1 0 0 L(1) + L(2) + L(3);
     0 1 0      0;
     0 0 1      0;
     0 0 0      1];
 
 
S = [0 0 0;
     0 0 0;
     1 1 1;
     0 0 0;
     0 -L(1) -L(1)-L(2);
     0 0 0;];
 
 B = S; % as we are calculating FK in space frame, Body frame twist vectors are of no use. 
 
 robot = defineRobot(M, S, B)

jointAngles = sym('theta', [robot.numJoints 1])

[T, jointToJointTransforms, err] = FK_space(robot,jointAngles)

T = simplify(T)

%% The answer below corresponds with the FK calculated in the book in equations 4.1 - 4.3 on pg 137.
% T =
%  
% [ cos(theta1 + theta2 + theta3), -sin(theta1 + theta2 + theta3), 0, L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)]
% [ sin(theta1 + theta2 + theta3),  cos(theta1 + theta2 + theta3), 0, L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)]
% [                             0,                              0, 1,                                                                           0]
% [                             0,                              0, 0,                                                                           1]
 