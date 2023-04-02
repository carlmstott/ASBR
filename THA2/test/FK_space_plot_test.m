%% This is a demo for the robot in Exercise 4.11 (Fig 4.17) on pg 162 of Modern Robotics by Lynch et al


clear; clc; close all;

L = [1, 2, 3];

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

%we need to define a theta vector here
[robot, err] = defineRobot(M, S, B);

jointAngles = randi([-314, 314], robot.numJoints, 1, 'double') / 100;

[T, jointToJointTransforms, spaceToJointTransforms, err] = FK_space(robot,jointAngles, true)
