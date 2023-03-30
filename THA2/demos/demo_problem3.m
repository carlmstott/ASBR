%% This is a demo for the robot in Exercise 5.12 of Modern Robotics by Lynch et al


M=0; %not using

syms L;
syms T4; %represents theta4

S=sym('S', [6,4]);

S(:,1)=[0;0;1;0;0;0];

S(:,2)=[1;0;0;0;L;0];

S(:,3)=[0;0;1;L;0;0];

S(:,4)=[0;0;0;0;1;0];

B=sym('B', [6,4]);

B(:,4)=[0;0;0;0;1;0];

B(:,3)=[0;0;1;0;0;0];

B(:,2)=[1;0;0;0;0;L];

B(:,1)=[0;0;1;-L;0;0];

[robot, err] = defineRobot(M, S, B);

jointAngles = sym('T', [robot.numJoints 1]);

Js = simplify(J_space(robot,jointAngles))

Jb = simplify(J_body(robot, jointAngles))