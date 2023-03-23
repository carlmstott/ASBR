%% This is a demo for a 3 DoF planar manipulator as shown in Fig 4.1 on pg 138 of Modern Robotics by Lynch et al

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
 
 B = S;
 
 robot = defineRobot(M, S, B)

jointAngles = sym('theta', [robot.numJoints 1])

[T, jointToJointTransforms, err] = FK_space(robot,jointAngles)

J = J_space(robot,jointAngles)