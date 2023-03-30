%demo for Kuka robot
clear all;
close all;


M=[1,0,0,0;0,1,0,1765;0,0,1,1910;0,0,0,1]; %unneeded

S=sym('S', [6,6]);

S(:,1)=[0;0;1;0;0;0];

S(:,2)=[-1;0;0;0;-645;330];

S(:,3)=[-1;0;0;0;-1910;330];

S(:,4)=[0;-1;0;-1910;0;0];

S(:,5)=[-1;0;0;0;-1910;1550];

S(:,6)=[0;1;0;-1910;0;0];


B=sym('B', [6,6]); %not done yet

[robot, err] = defineRobot(M, S, B);

jointAngles = sym('T', [robot.numJoints 1]);
