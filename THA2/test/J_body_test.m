clear all;
close all;


%this script is used to test the FK_body function using the example
%provided on the modern robotics github page:


syms L1 L2

M=eye(4); %we dont need this, so I'm leaving it empty


% S = [0 0    0       0;
%      0 0    0       0;
%      1 1    1       0;
%      0 0    0       0;
%      0 -L1 -L1-L2   0;
%      0 0    0       1;];
%  
%  B = S; % I am only using symbolic funconality to see if symbolic functonality works,
%  %it does not have to be correct.
%  
% robot = defineRobot(M, S, B);
% 
% jointAngles = sym('theta', [robot.numJoints 1]);
% 
% J = J_space(robot,jointAngles)


% The below is the neumeric test I used, gotten from the 
%https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/JacobianBody.m
Blist = [[0; 0; 1;   0; 0.2; 0.2], ...
        [1; 0; 0;   2;   0;   3], ...
        [0; 1; 0;   0;   2;   1], ...
        [1; 0; 0; 0.2; 0.3; 0.4]];

S=Blist; %doesent matter what this 

jointAngles = [0.2; 1.1; 0.1; 1.2];


robot=defineRobot(M, S, Blist);

J=J_body(robot,jointAngles);

%our J_body function returns the correct solution, see above github link to
%compare 

% J =
% 
%    -0.0453    0.9950         0    1.0000
%     0.7436    0.0930    0.3624         0
%    -0.6671    0.0362   -0.9320         0
%     2.3259    1.6681    0.5641    0.2000
%    -1.4432    2.9456    1.4331    0.3000
%    -2.0664    1.8288   -1.5887    0.4000