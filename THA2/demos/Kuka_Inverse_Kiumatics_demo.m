clear all;
close all;
%Kuka Robot IK Test
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

%choose the below T, the inverse kinumatics algorithem will attempt to make
%the kuka reach it!
T=[1   0    0     -10
   0   1    0     10
    0  0    1     10
    0   0    0    1
];

%pick max iteration amount
iterations=100;

%pick orientation stopping criteria (error)
OSC=.1;
    
%Pick Translatonal stopping criteria (error)
TSC=.1;

%pick if you want the IK function to plot everything (not reccomended for
%speed)
%plot=0;

%robot starts at near base position, but not at base position becasue thats
%in singularity
EndConfig=J_inverse_kinumatics_Kuka(robot, [0;0;0;0;0;0], T, iterations, OSC, TSC);

%now we plot the kuka to see how close it got to the desired end position
[T, jointToJointTransforms, err] = Fk_Space_for_Kuka(robot,EndConfig, 0);

