%this script is used to test the FK_body function using the example
%provided on the modern robotics github page:
%(https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/FKinBody.m

clear all;
close all;


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


%Test is conducted using the values 


 
 B = S; % as we are calculating FK in space frame, Body frame twist vectors are of no use. 
 
robot = defineRobot(M, S, B)

jointAngles = sym('theta', [robot.numJoints 1])

jointAngles=[1,2,4];






%the below is a neumeric example to test the FK_body function
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% 
% Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
% 
% jointAngles = [pi/2; 0; pi/2];
% 
% S=Blist; %the S entry in define robot is used to track the fwd kinumatics,which we are not using
% 
% robot=defineRobot(M, S, Blist);


[BodyK, jointTransformations]=FK_body(robot,jointAngles, 1); %change to one if symbolic



% hold on
% 
% jointangles = [pi / 2; 3; 5];
% [BodyK, jointTransformations]=FK_body(robot,jointangles);

%expected output is below, which is the same as the outcome provided in the
%example. This is a good test becasue it contains both prismatic and
%revelute joints.

% bodyK =
% 
%    -0.0000    1.0000         0   -5.0000
%     1.0000    0.0000         0    4.0000
%          0         0   -1.0000    1.6858
%          0         0         0    1.0000