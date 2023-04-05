%Inverse kinumatics test function, using 3DOF planer maniuplator

clear all;
close all; clc;

L = [1,2,3];

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
 
 B =[0,0,0;
     0,0,0;
     1,1,1;
     0,0,0;
     L(1)+L(2)+L(3),L(2)+L(3),L(3);
     0,0,0];
 
 robot = defineRobot(M, S, B);

jointAngles = [pi/4;pi/2;0]; %joint angles used to generate a reachable T which
                            %will be the goal of our FK function

[T, err] = FK_space(robot,jointAngles, 1); %this gives us the T we want to 
                                           %reach with our FK function

EndConfig=J_inverse_kinumatics(robot, [pi-.1;pi/2;0], T);

[TafterIK, err] = FK_space(robot,EndConfig, 1);
