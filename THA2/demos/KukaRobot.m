%demo for Kuka robot
clear all;
close all;  



%dimentions are in CM.
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


[robot, err] = defineRobot(M, S, B);

jointAngles = [0;0;0;0;0;0];


[T, jointToJointTransforms, err] = Fk_Space_for_Kuka(robot,jointAngles, 0)
