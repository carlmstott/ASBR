% This script tests the DLS_inverse_kinematics using the Kuka KR120 robot.
% The initial configuration input to the IK solver is a singulaarity. The
% Solver detects this by calculating the isotropy (> inf at singularity)
% and switches to using the Damped Least Squares algorithm. Once out of
% singularity, it switches to using Newton-Raphson method to get the IK
% solution"

clear all;
close all; clc;

% Screw Axes for a Kuka KR120
M(:,:,1)=[1,0,0,0
      0,1,0,0
      0,0,1,0
      0,0,0,1];

M(:,:,2)=[1,0,0,0
          0,1,0,3.30
          0,0,1,4.65
          0,0,0,1];

M(:,:,3)=[1,0,0,0
          0,1,0,3.30
          0,0,1,17.95
          0,0,0,1];

M(:,:,4)=[1,0,0,0
          0,1,0,9.40
          0,0,1,19.10
          0,0,0,1];

M(:,:,5)=[1,0,0,0
          0,1,0,15.50
          0,0,1,19.10
          0,0,0,1];


M(:,:,6)=[1,0,0,0
          0,1,0,17.65
          0,0,1,19.10
          0,0,0,1];


S=sym('S', [6,6]);

S(:,1)=[0; 0; 1; 0; 0; 0];

S(:,2)=[-1; 0; 0; 0; -6.45; 3.30];

S(:,3)=[-1; 0; 0; 0; -19.10; 3.30];

S(:,4)=[0; 1; 0; -19.10; 0; 0];

S(:,5)=[-1; 0; 0; 0; -19.10; 15.50];

S(:,6)=[0; 1; 0; -19.10; 0; 0];



B=sym('B', [6,6]);

B(:,1)=[0; 0; 1; 0; -17.65; 0];

B(:,2)=[-1; 0; 0; 0; 12.65; -14.35];

B(:,3)=[-1; 0; 0; 0; 0; -14.35];

B(:,4)=[0; 1; 0; 0; 0; 0];

B(:,5)=[-1; 0; 0; 0; 0; -2.15];

B(:,6)=[0; 1; 0; 0; 0; 0];

B=double(B);

S=double(S);
 
robot = defineRobot(M, S, B);

jointAngles = randi([-243, -9], robot.numJoints, 1) / 100; %joint angles used to generate a reachable T which
                            %will be the goal of our FK function

%below is an example target end effector pose that was randomly generated
% T=[0.3161   -0.9272    0.2011   -7.8102
%     0.8429    0.3717    0.3890    7.1492
%     -0.4354    0.0465    0.8990   16.0442
%     0         0         0    1.0000]

jointAngles = [-1.86; -0.59; -1.7453; -1.51; -2.43; -1.92];

[T, err] = FK_space(robot,jointAngles, 0); %this gives us the T we want to 
                                           %reach with our FK function
T = [0.7972          -0.4603         0.3907        -13.2557;
      0.5705         0.7860       -0.2382       8.5137;
       -0.1974       0.4129         0.8891        24.5323;
         0       0                     0            1]
Input_Joint_Angles = jointAngles

[currJointAngles, allJacobians, allNormOrient, allNormTrans, err] = constrainedIK(robot, jointAngles, T, 500,0.1, 0.01, false);

Output_Joint_Angles = currJointAngles 

[TafterIK, err] = FK_space(robot,currJointAngles, 0);

T

TafterIK

ellipsoidPlotter(allJacobians, allNormOrient, allNormTrans)
