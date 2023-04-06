%Inverse kinumatics test function, using 3DOF planer maniuplator

clear all;
close all; clc;

L = [1,2,3];

% M = [1 0 0 L(1) + L(2) + L(3);
%      0 1 0      0;
%      0 0 1      0;
%      0 0 0      1];
% 
% 
% S = [0 0 0;
%      0 0 0;
%      1 1 1;
%      0 0 0;
%      0 -L(1) -L(1)-L(2);
%      0 0 0;];
% 
%  B =[0,0,0;
%      0,0,0;
%      1,1,1;
%      0,0,0;
%      L(1)+L(2)+L(3),L(2)+L(3),L(3);
%      0,0,0];

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

jointAngles = randi([-314, 314], robot.numJoints, 1) / 100; %joint angles used to generate a reachable T which
                            %will be the goal of our FK function

[T, err] = FK_space(robot,jointAngles, 0); %this gives us the T we want to 
                                           %reach with our FK function

EndConfig=IK(robot, randi([-314, 314], robot.numJoints, 1) / 100, T, 500);

[TafterIK, err] = FK_space(robot,EndConfig, 0);
